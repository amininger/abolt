package abolt.arm;

/**
 * ArmGUI displays the current state of the 6DOF arl arm and allows
 * manual control of the joint angles when desired.
 *
 * The main function of this class is to display the arm given by the
 * angles in dynamixel_status_list_t (which are sent from
 * arlstaff.arm.ArmDriver.java).  The alternate function is to allow
 * manual control (via sliders) of the servo joint angles.
 *
 * The LCM interface used to talk to arlstaff.arm.ArmDriver is defined
 * in the lcmtypes: dynamixel_status_list_t and
 * dynamixel_command_list_t.
 **/
import java.awt.*;
import javax.swing.*;
import java.io.*;
import java.util.*;

import lcm.lcm.*;

import april.dynamixel.*;
import april.util.*;
import april.vis.*;
import april.jmat.*;

import abolt.lcmtypes.*;

public class ArmGUI implements LCMSubscriber
{
    VisWorld vw = new VisWorld();
    VisLayer vl = new VisLayer(vw);
    VisCanvas vc = new VisCanvas(vl);

    static LCM lcm = LCM.getSingleton();

    ExpiringMessageCache<dynamixel_status_list_t> qdslist;
    static final int MAX_NUM_UTIMES = 100;
    int numUtimes = 0;
    int idxUtime = 0;
    long[] msgUtimes = new long[MAX_NUM_UTIMES];

    // PLASTIC
    static final Color colorPlastic = new Color(0x08, 0x08, 0x48);
    static final double[] SIZE_PLASTIC_BASE = {0.052,  0.04,   0.039};
    static final double[] SIZE_PLASTIC_0_1 =  {0.074,  0.024,  0.003};
    static final double[] SIZE_PLASTIC_1_2 =  {0.07,   0.024,  0.003};
    static final double[] SIZE_PLASTIC_2_3 =  {0.036,  0.024,  0.003};
    static final double[] SIZE_FINGER =       {0.008,  0.061,  0.008};

    // Single direction translation vectors from each servo to next (and origin to 0)
    static final double TRANS_O_0 = 0.055;    // origin_to_servo0
    static final double TRANS_0_1 = 0.063;    // 118 from origin
    static final double TRANS_1_2 = 0.10075;  // 120.45 - 19.75 (subtracting screw pattern)
    static final double TRANS_2_3 = 0.1;      // 11975 - 19.75
    static final double TRANS_3_4 = 0.055625; // 84.5 - 19.75/2 - 38/2 =
    static final double TRANS_4_5 = 0.04;     // 69 - 32/2 - 26/2

    // circular horn pieces (aka. axis attachment points)
    static final double HORN_RADIUS = 0.011;
    static final VisObject servoAxisHorn  = new VzCylinder(HORN_RADIUS, 0.002,
                                                           new VzMesh.Style(Color.lightGray));
    static final VisObject servoOtherHorn = new VzCylinder(HORN_RADIUS, 0.002,
                                                           new VzMesh.Style(Color.darkGray));

    // with servo origin on axis center (mid-body)
    static final double[] SIZE_AX12_BOX =       { 0.05,  0.032, 0.038 };
    static final double[] SIZE_MX28_BOX =       { 0.05,  0.035, 0.03 };
    static final double[] SIZE_MX28_INNER_BOX = { 0.045, 0.024, 0.036 };
    static final double OFFSET_X_BOX_AX12 = 0.014;
    static final double OFFSET_X_BOX_MX28 = 0.0125;

    // static servo Objects
    static final VisObject servoMX28 = makeMX28(Color.black);
    static final VisObject servoMX28Alarm = makeMX28(Color.red);
    static final VisObject servoAX12 = makeAX12(Color.black);
    static final VisObject servoAX12Alarm = makeAX12(Color.red);

    /**
     * Constructor takes parameterGUI.  Calling class (or main)
     * handles JFrame, location on screen, etc.
     **/
    public ArmGUI(ParameterGUI pg)
    {
        vl.backgroundColor = Color.white;

        qdslist = new ExpiringMessageCache<dynamixel_status_list_t>(0.25);

        ManualControlThread controlThread = new ManualControlThread(pg);

        ((DefaultCameraManager) vl.cameraManager).uiLookAt(new double[] { -0.3127,  -.29519, .31082 },
                                                           new double[] {   .12172,  .12318, 0 },
                                                           new double[] {  0.32996,  .31778, .8889 }, true);
        lcm.subscribe("ARM_STATUS", this);
        new DrawThread().start();
    }

    public VisCanvas getCanvas()
    {
        return vc;
    }

    private class ManualControlThread extends Thread implements ParameterListener
    {
        double[] desiredAngles;
        ParameterGUI pg;

        ManualControlThread(ParameterGUI pg)
        {
            this.pg = pg;

            desiredAngles = new double[6];
            pg.addCheckBoxes("enableManual", "Enable Manual Mode", false);
            for (int i = 0; i < 3; i++) {
                pg.addDoubleSlider("angle-" + i, "Servo "+i+" Angle (MX28)", -180, 179.9999, desiredAngles[i]);
                pg.setEnabled("angle-"+i, false);
            }
            for (int i = 3; i < 6; i++) {
                pg.addDoubleSlider("angle-" + i, "Servo "+i+" Angle (AX12)", -150, 150, desiredAngles[i]);
                pg.setEnabled("angle-"+i, false);
            }
            pg.addButtons("home", "GoTo Home");
            pg.addListener(this);
            start();
        }

        public void parameterChanged(ParameterGUI pg, String name)
        {
            if (name.equals("home")) {
                pg.sb("enableManual", true);
                for (int i = 0; i < 6; i++) {
                    pg.setEnabled("angle-"+i, true);
                    pg.sd("angle-"+i, 0);
                }
            } else if (name.equals("enableManual")) {
                for (int i = 0; i < 6; i++) {
                    pg.setEnabled("angle-"+i, pg.gb("enableManual"));
                }
            }
        }

        public void updateSliders(dynamixel_status_list_t dslist)
        {
            if (dslist == null)
                return;
            for (int i = 0; i < 6; i++)
                pg.sd("angle-"+i, Math.toDegrees(dslist.statuses[i].position_radians));
        }

        public void sendCommandMessage()
        {
            long now = TimeUtil.utime();

            dynamixel_command_list_t cmdlist = new dynamixel_command_list_t();
            cmdlist.len = 6;
            cmdlist.commands = new dynamixel_command_t[cmdlist.len];
            for (int i = 0; i < 6; i++) {
                dynamixel_command_t cmd = new dynamixel_command_t();
                cmd.position_radians = Math.toRadians(pg.gd("angle-"+i));
                cmd.utime = now;
                cmd.speed = 0.5;
                cmd.max_torque = 0.5;
                cmdlist.commands[i] = cmd;
            }
            lcm.publish("ARM_COMMAND", cmdlist);
        }

        public void run()
        {
            while (true) {
                if (pg.gb("enableManual")) {
                    sendCommandMessage();
                    TimeUtil.sleep(50);
                } else {
                    updateSliders(qdslist.get());
                    TimeUtil.sleep(10);
                }
            }
        }
    }

    public void messageReceived(LCM lcm, String channel, LCMDataInputStream ins)
    {
        try {
            messageReceivedEx(lcm, channel, ins);
        } catch (IOException ex) {
            System.out.println("ex: "+ex);
        }
    }

    public void messageReceivedEx(LCM lcm, String channel, LCMDataInputStream ins) throws IOException
    {
        if (channel.equals("ARM_STATUS")) {
            dynamixel_status_list_t dslist = new dynamixel_status_list_t(ins);
            qdslist.put(dslist, dslist.statuses[0].utime);

            synchronized(msgUtimes) {
                numUtimes = Math.min(MAX_NUM_UTIMES, ++numUtimes);
                idxUtime = ++idxUtime % MAX_NUM_UTIMES;
                msgUtimes[idxUtime] = TimeUtil.utime();
            }
        }
    }

    static VisObject makeAX12(Color color)
    {
        return new VisChain(
            new VisChain(LinAlg.translate(-OFFSET_X_BOX_AX12, 0, 0),
                         new VzBox(SIZE_AX12_BOX,new VzMesh.Style(color))),
            new VisChain(LinAlg.translate(0, 0, 0.002 + SIZE_AX12_BOX[2]/2), servoAxisHorn));
    }

    static VisObject makeMX28(Color color)
    {
        return new VisChain(
            new VisChain(LinAlg.translate(-OFFSET_X_BOX_MX28, 0, 0),
                         new VzBox(SIZE_MX28_BOX, new VzMesh.Style(color)),
                         new VzBox(SIZE_MX28_INNER_BOX, new VzMesh.Style(color))),
            new VisChain(LinAlg.translate(0, 0, 0.002 + SIZE_MX28_INNER_BOX[2]/2), servoAxisHorn));
    }

    void drawArm(dynamixel_status_list_t dslist)
    {
        VisWorld.Buffer vb;


        vb = vw.getBuffer("time");
        long utime = TimeUtil.utime();
        int SECONDS = 3;
        long threshTime = utime - 1000000*SECONDS;
        int num = 0;
        String str = null;

        if (numUtimes > 0) {
            long dt;
            synchronized(msgUtimes) {
                dt = (utime - msgUtimes[idxUtime])/1000;
                for (int i = 0; i < numUtimes; i++)
                    if (msgUtimes[i] > threshTime)
                        num++;
            }
            int hz = (int) Math.round(num*1.0/SECONDS);
            str = String.format("Status received %3d ms ago (~%2d Hz)", dt, hz);
        }
        if (num == 0)
            str = "No status received yet " + this.numUtimes;
        vb.addBack(new VisPixCoords(VisPixCoords.ORIGIN.BOTTOM_LEFT,
                                    new VzText(VzText.ANCHOR.BOTTOM_LEFT,
                                               "<<dropshadow=#00330088,monospaced-16-bold,green>>" +
                                               str)));
        vb.swap();

        vb = vw.getBuffer("arm");
        if (dslist != null && dslist.len == 6) {
            vb.addBack(new VisChain(
                           // We'll set up every stage as pointing in the +z direction

                           // base plastic
                           new VisChain(LinAlg.translate(-OFFSET_X_BOX_MX28, 0, SIZE_PLASTIC_BASE[2]/2),
                                        new VzBox(SIZE_PLASTIC_BASE, new VzMesh.Style(colorPlastic))),

                           // servo 0
                           LinAlg.translate(0, 0, TRANS_O_0),
                           (dslist.statuses[0].error_flags == 0 ? servoMX28 : servoMX28Alarm),
                           LinAlg.rotateZ(dslist.statuses[0].position_radians),

                           // servo 1
                           LinAlg.translate(0, 0, TRANS_0_1),
                           LinAlg.rotateX(Math.PI/2), LinAlg.rotateZ(Math.PI/2),
                           (dslist.statuses[1].error_flags == 0 ? servoMX28 : servoMX28Alarm),
                           LinAlg.rotateZ(dslist.statuses[1].position_radians),

                           // servo 2
                           makeSymmetricArmLink(SIZE_MX28_INNER_BOX[2]/2, SIZE_PLASTIC_0_1),
                           LinAlg.translate(TRANS_1_2, 0, 0),
                           (dslist.statuses[2].error_flags == 0 ? servoMX28 : servoMX28Alarm),
                           LinAlg.rotateZ(dslist.statuses[2].position_radians),

                           // servo 3
                           makeSymmetricArmLink(SIZE_MX28_INNER_BOX[2]/2, SIZE_PLASTIC_1_2),
                           LinAlg.translate(TRANS_2_3, 0, 0),
                           (dslist.statuses[3].error_flags == 0 ? servoAX12 : servoAX12Alarm),
                           LinAlg.rotateZ(dslist.statuses[3].position_radians),

                           // servo 4
                           makeSymmetricArmLink(SIZE_AX12_BOX[2]/2, SIZE_PLASTIC_2_3),
                           new VisChain(LinAlg.translate(SIZE_PLASTIC_2_3[0], 0, 0),
                                        new VzBox(0.003, 0.024, SIZE_AX12_BOX[2] + 0.012,
                                                  new VzMesh.Style(colorPlastic))),
                           LinAlg.translate(TRANS_3_4, 0, 0), LinAlg.rotateY(Math.PI/2),
                           (dslist.statuses[4].error_flags == 0 ? servoAX12 : servoAX12Alarm),
                           LinAlg.rotateZ(dslist.statuses[4].position_radians),

                           // servo 5
                           LinAlg.translate(0,0, TRANS_4_5), LinAlg.rotateX(-Math.PI/2),
                           LinAlg.rotateZ(Math.PI), LinAlg.translate(OFFSET_X_BOX_AX12, 0, 0),
                           (dslist.statuses[5].error_flags == 0 ? servoAX12 : servoAX12Alarm),

                           // stationary gripper
                           new VisChain(LinAlg.translate(-0.0225, 0.014, 0),
                                        new VzBox(0.022, 0.009, SIZE_AX12_BOX[2]+0.006,
                                                  new VzMesh.Style(colorPlastic)),
                                        LinAlg.rotateZ(Math.PI/36),  // 5 degrees
                                        LinAlg.translate(0, SIZE_FINGER[1]/2, 0.01),
                                        new VzBox(SIZE_FINGER, new VzMesh.Style(colorPlastic)),
                                        LinAlg.translate(0, 0, -0.020),
                                        new VzBox(SIZE_FINGER, new VzMesh.Style(colorPlastic))),

                           // moving gripper
                           LinAlg.rotateZ(dslist.statuses[5].position_radians),
                           new VisChain(new VisChain(
                                            LinAlg.translate(0,0,SIZE_AX12_BOX[2]/2+0.0045),
                                            new VzCylinder(HORN_RADIUS, 0.003,
                                                           new VzMesh.Style(colorPlastic)),
                                            new VisChain(LinAlg.translate(0.009, -0.0075, 0),
                                                         LinAlg.rotateZ(-Math.PI/12),
                                                         new VzBox(0.018, 0.01, 0.003,
                                                                   new VzMesh.Style(colorPlastic))),

                                            LinAlg.translate(0,0,-2*(SIZE_AX12_BOX[2]/2+0.0045)),
                                            new VzCylinder(HORN_RADIUS, 0.003,
                                                           new VzMesh.Style(colorPlastic)),
                                            new VisChain(LinAlg.translate(0.009, -0.0075, 0),
                                                         LinAlg.rotateZ(-Math.PI/12),
                                                         new VzBox(0.018, 0.01, 0.003,
                                                                   new VzMesh.Style(colorPlastic)))),
                                        LinAlg.translate(0.017, -0.01, 0),
                                        new VzBox(0.003, 0.01, SIZE_AX12_BOX[2]+0.012,
                                                  new VzMesh.Style(colorPlastic)),
                                        LinAlg.rotateZ(-(Math.PI/2 + Math.PI/36)),   // 5 degrees
                                        LinAlg.translate(0, SIZE_FINGER[1]/2, 0),
                                        new VzBox(SIZE_FINGER, new VzMesh.Style(colorPlastic)),
                                        LinAlg.translate(0, 0, 0.020),
                                        new VzBox(SIZE_FINGER, new VzMesh.Style(colorPlastic)),
                                        LinAlg.translate(0, 0, -0.040),
                                        new VzBox(SIZE_FINGER, new VzMesh.Style(colorPlastic)))
                           ));
        }
        vb.swap();
    }

    VisObject makeSymmetricArmLink(double zRotationEnd, double []sizeWall)
    {
        VzMesh.Style style = new VzMesh.Style(colorPlastic);
        return new VisChain(
            new VisChain(LinAlg.translate(0, 0, zRotationEnd + sizeWall[2]/2 + 0.003),
                         new VzCylinder(sizeWall[1]/2, sizeWall[2]-0.00001, style),
                         LinAlg.translate(sizeWall[0]/2, 0, 0),
                         new VzBox(sizeWall, style)),
            new VisChain(LinAlg.translate(0, 0, -(zRotationEnd + 0.002)),
                         servoOtherHorn,
                         LinAlg.translate(0, 0, -(sizeWall[2]/2 + 0.001)),
                         new VzCylinder(sizeWall[1]/2, sizeWall[2]-0.00001, style),
                         LinAlg.translate(sizeWall[0]/2, 0, 0),
                         new VzBox(sizeWall, style))
            );
    }

    public class DrawThread extends Thread
    {
        public void run()
        {
            VisWorld.Buffer vb = vw.getBuffer("board");
            vb.addBack(new VzRectangle(24*0.0254, 24*0.0254, new VzMesh.Style(Color.darkGray)));
            vb.swap();

            vb = vw.getBuffer("origin");
            vb.addBack(new VisChain(LinAlg.scale(0.05, 0.05, 0.05),
                                    new VzAxes()));
            vb.swap();

            while (true) {
                drawArm(qdslist.get());
                TimeUtil.sleep(100);
            }
        }
    }

    public static void main(String args[]) throws IOException
    {
        JFrame jf = new JFrame("ArmGUI");
        ParameterGUI pg = new ParameterGUI();
        jf.setLayout(new BorderLayout());
        jf.add(pg, BorderLayout.SOUTH);
        jf.setSize(800, 600);
        jf.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        jf.setVisible(true);
        jf.add(new ArmGUI(pg).getCanvas(), BorderLayout.CENTER);
    }
}
