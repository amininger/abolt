package abolt.arm;

import java.util.Random;
import java.awt.*;
import java.awt.event.*;
import javax.swing.*;
import java.io.*;

import lcm.lcm.*;

import april.jmat.geom.*;
import april.jmat.*;
import april.util.*;
import april.vis.*;

import abolt.lcmtypes.*;

public class ArmController extends Thread
{
    VisWorld vw = new VisWorld();
    VisLayer vl = new VisLayer(vw);
    VisCanvas vc = new VisCanvas(vl);

    static LCM lcm = LCM.getSingleton();

    ParameterGUI pgArm, pgClick;

    // goal location
    double goalXY[] = null;

    static boolean verbose = false;

    public ArmController(ParameterGUI pgArm, ParameterGUI pgClick)
    {
        this.pgArm = pgArm;
        this.pgClick = pgClick;

        double dim = -12 * 0.0254;
        ((DefaultCameraManager) vl.cameraManager).interfaceMode = 2.0;
        ((DefaultCameraManager) vl.cameraManager).fit2D(new double[] { -dim, -dim },
                                                        new double[] {  dim,  dim },
                                                        true);

        VisWorld.Buffer vb = vw.getBuffer("hud");
        int t = 43;
        int b = 3;
        double p = b*b/(t/30.0 + 1);
        String format = "<<dropshadow=#FFFFFFFF, large>>";
        vb.addBack(new VisPixCoords(
                                    VisPixCoords.ORIGIN.TOP_LEFT,
                                    new VzText(VzText.ANCHOR.TOP_LEFT, format+
                                               "<<red>>Objects Found" +
                                               "\n<<left>>Total: <<right>>"+b +
                                               "\nBalls: <<right>>2" +
                                               "\nSmallBlocks: <<right>>0" +
                                               "\nLargeBlocks: <<right>>1")));
        vb.addBack(new VisPixCoords(
                                    VisPixCoords.ORIGIN.TOP,
                                    new VzText(VzText.ANCHOR.TOP, format+"<<red>>Points Acrued\n"+p)));
        vb.addBack(new VisPixCoords(
                                    VisPixCoords.ORIGIN.TOP_RIGHT,
                                    new VzText(VzText.ANCHOR.TOP_RIGHT, format+"<<red>>Time Elapsed\n<<right>>"+t+" s")));
        vb.swap();

        vb = vw.getBuffer("balls");
        Random r = new Random();
        for (int i = 0; i < 10; i++) {
            double x = 2*(r.nextDouble()-0.5)*dim;
            double y = 2*(r.nextDouble()-0.5)*dim;
            vb.addBack(new VisChain(LinAlg.translate(x, y, 0.01),
                                    new VzCircle(0.02, new VzMesh.Style(Color.yellow))));
            if (i == 0)
                vb.addBack(new VisChain(LinAlg.translate(x, y, 0.01),
                                        new VzRectangle(0.05, 0.05, new VzLines.Style(Color.green, 3))));

        }
        vb.swap();

        pgClick.addCheckBoxes("enableClick", "Enable Click Mode", true);
        pgClick.addDoubleSlider("height", "Height", 0, 0.12, 0.05);

        pgClick.addListener(new ParameterListener() {
            public void parameterChanged(ParameterGUI pg, String name)
        {
            ParameterGUI pgArm = ArmController.this.pgArm;
            if (name.equals("enableClick") && pg.gb("enableClick")) {
                pgArm.sb("enableManual", false);

                // need to duplicate this here because do not want to set notifyOnSet parameter
                for (int i = 0; i < 6; i++) {
                    pgArm.setEnabled("angle-"+i, false);
                }
            }
        }
        });

        pgArm.addListener(new ParameterListener() {
            public void parameterChanged(ParameterGUI pg, String name)
        {
            ParameterGUI pgClick = ArmController.this.pgClick;
            if (name.equals("enableManual") && pg.gb("enableManual")) {
                pgClick.sb("enableClick", false);
            } else if (name.equals("home")) {
                pgClick.sb("enableClick", false);
            }
        }
        });
        vl.addEventHandler(new MyEventHandler());
    }

    public VisCanvas getCanvas()
    {
        return vc;
    }

    public void drawClickWorld(double xy[], boolean enabled)
    {
        VisWorld.Buffer vb;
        double radius = 0, angle = 0;
        Color color = enabled ? Color.green : Color.red;

        vb = vw.getBuffer("goal");
        if (xy != null) {
            radius = LinAlg.magnitude(xy);
            angle = Math.atan2(xy[1], xy[0]);
            vb.addBack(new VisChain(LinAlg.translate(xy),
                                    new VzCircle(0.0025, new VzMesh.Style(color)),
                                    new VzCircle(0.01, new VzLines.Style(color, 2)),
                                    new VzCircle(0.02, new VzLines.Style(color, 1))
                                   ));
        }
        vb.swap();

        vb = vw.getBuffer("text");
        String str = "<<monospaced-16-bold,black>>" +
            (xy == null ? "No Goal Set" :
             String.format("Goal Set to (%3.3f, %3.3f)\nZ:%3.3f\tradius:%3.3f\tangle:%3.3f",
                           xy[0], xy[1], pgClick.gd("height"), radius, angle));
        vb.addBack(new VisPixCoords(
                                    VisPixCoords.ORIGIN.BOTTOM_RIGHT,
                                    new VzText(VzText.ANCHOR.BOTTOM_RIGHT, str)));
        vb.swap();
    }

    static public double[] getServoAnglesGeometric(double xy[], double zEndEffector)
    {
        if (zEndEffector < 0) {
            System.out.println("Invalid z height");
            return null;
        }
        // distance from servo 3 axis (the last of the 3 parallel servos axis)
        final double eeLength = (ArmGUI.TRANS_3_4 + ArmGUI.TRANS_4_5 +
                                 ArmGUI.SIZE_FINGER[1]*Math.cos(Math.PI/36));
        double angles[] = new double[6];

        double r = LinAlg.magnitude(xy);
        angles[0] = Math.atan2(xy[1], xy[0]);

        // z of S3 w.r.t S1's home position
        double z = zEndEffector + eeLength - (ArmGUI.TRANS_O_0 + ArmGUI.TRANS_0_1);

        double distS3 = Math.sqrt(r*r + z*z);
        final double MAX_DIST = ArmGUI.TRANS_1_2 + ArmGUI.TRANS_2_3;
        if (distS3 > MAX_DIST) {
            System.out.println("Position out of range (for vertical end effector)");
            return null;
        }

        // use cosine law to define 3 angles for elbow up triangle.
        // cos(theta_c) = (a^2 + b^2 - c^2) / 2ab
        double a1 = Math.acos((ArmGUI.TRANS_1_2 * ArmGUI.TRANS_1_2 +
                               distS3 * distS3 -
                               ArmGUI.TRANS_2_3 * ArmGUI.TRANS_2_3) / (2 * ArmGUI.TRANS_1_2 * distS3));
        double a2 = Math.acos((ArmGUI.TRANS_1_2 * ArmGUI.TRANS_1_2 +
                               ArmGUI.TRANS_2_3 * ArmGUI.TRANS_2_3 -
                               distS3 * distS3) / (2 * ArmGUI.TRANS_1_2 * ArmGUI.TRANS_2_3));
        double a3 = Math.PI - a1 - a2;

        double atan_z_r = Math.atan2(z,r);
        angles[1] = atan_z_r + a1 - Math.PI / 2;
        angles[2] = a2 - Math.PI;
        angles[3] = a3 - atan_z_r - Math.PI / 2;
        angles [5] = Math.PI / 2;

        return angles;
    }

    static public void sendCommandMessage(double xy[], double z, boolean enableSend)
    {
        double angles[] = getServoAnglesGeometric(xy, z);
        if (angles == null)
            return;

        long now = TimeUtil.utime();
        dynamixel_command_list_t cmdlist = new dynamixel_command_list_t();
        cmdlist.len = 6;
        cmdlist.commands = new dynamixel_command_t[cmdlist.len];
        for (int i = 0; i < 6; i++) {
            if (verbose)
                System.out.printf("%3.3f\t", angles[i]);
            dynamixel_command_t cmd = new dynamixel_command_t();
            cmd.position_radians = enableSend ? MathUtil.mod2pi(angles[i]) : 0;
            cmd.utime = now;
            cmd.speed = 0.5;
            cmd.max_torque = 0.6;
            cmdlist.commands[i] = cmd;
        }
        if (verbose)
            System.out.println();
        lcm.publish("ARM_COMMAND", cmdlist);
    }

    public void run()
    {
        VzGrid.addGrid(vw, new VzGrid(new VzMesh.Style(new Color(200,200,200)),
                                      new VzLines.Style(new Color(50,50,50), 1)));
        VisWorld.Buffer vb = vw.getBuffer("board");
        vb.addBack(new VzRectangle(24*0.0254, 24*0.0254, new VzMesh.Style(Color.darkGray)));
        vb.swap();

        vb = vw.getBuffer("origin");
        vb.addBack(new VisChain(LinAlg.scale(0.05, 0.05, 0.05),
                                new VzAxes()));
        vb.swap();

        while(true) {
            TimeUtil.sleep(200);

            boolean enabled = pgClick.gb("enableClick");
            double xy[] = goalXY;

            drawClickWorld(xy, enabled);
            if (enabled && xy != null) {
                sendCommandMessage(xy, pgClick.gd("height"), true);
            }
        }
    }

    class MyEventHandler extends VisEventAdapter
    {
        /** Return true if you've consumed the event. **/
        public boolean mouseClicked(VisCanvas vc, VisLayer vl, VisCanvas.RenderInfo rinfo,
                                    GRay3D ray, MouseEvent e)
        {
            if (e.getButton() == MouseEvent.BUTTON1) {
                double xy[] = ray.intersectPlaneXY(0);
                goalXY = xy;
            }
            return true;
        }
    }

    public static void main(String args[])
    {
        int WIN_WIDTH = 1000;

        JFrame jf = new JFrame("Arm Controller");

        ParameterGUI pgArm   = new ParameterGUI();
        ParameterGUI pgClick = new ParameterGUI();

        ArmGUI armOnlyGUI = new ArmGUI(pgArm);
        ArmController cntrlr = new ArmController(pgArm, pgClick);

        jf.setLayout(new BorderLayout());
        jf.setSize(WIN_WIDTH, 600);
        jf.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);

        JPanel jpArm = new JPanel();
        jpArm.setLayout(new BorderLayout());
        jpArm.add(armOnlyGUI.getCanvas(), BorderLayout.CENTER);
        jpArm.add(pgArm, BorderLayout.SOUTH);

        JPanel jpClick = new JPanel();
        jpClick.setLayout(new BorderLayout());
        jpClick.add(cntrlr.getCanvas(), BorderLayout.CENTER);
        jpClick.add(pgClick, BorderLayout.SOUTH);

        JSplitPane jsp = new JSplitPane(JSplitPane.HORIZONTAL_SPLIT, jpArm, jpClick);
        jsp.setDividerLocation(WIN_WIDTH/2);
        jsp.setResizeWeight(0.5);
        jf.add(jsp, BorderLayout.CENTER);

        jf.setVisible(true);
        cntrlr.run();
    }
}
