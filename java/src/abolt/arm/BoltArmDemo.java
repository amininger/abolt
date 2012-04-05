package abolt.arm;

import java.awt.*;
import java.awt.event.*;
import javax.swing.*;
import java.util.*;
import java.io.*;

import lcm.lcm.*;

import april.dynamixel.*;
import april.jserial.*;
import april.jmat.*;
import april.jmat.geom.*;
import april.vis.*;
import april.util.*;

import abolt.lcmtypes.*;

public class BoltArmDemo implements LCMSubscriber
{
    // LCM
    LCM lcm = LCM.getSingleton();

    // Arm joints
    ArrayList<Joint> joints;

    // Rendering thread
    RenderThread rt;
    KinematicsThread kt;

    // Cached arm information for planning
    double[] l;
    double baseHeight = 0.075;

    // Command line flags/options
    GetOpt opts;

    public BoltArmDemo(GetOpt opts_)
    {
        opts = opts_;

        // Initialize BOLT arm
        initArm();

        lcm.subscribe("ARM_STATUS", this);

        rt = new RenderThread();
        kt = new KinematicsThread();
        rt.start();
        kt.start();
    }

    private void initArm()
    {
        joints = new ArrayList<Joint>();
        RevoluteJoint j0, j1, j2, j3, j4, j5;
        RevoluteJoint.Parameters p0, p1, p2, p3, p4, p5;
        l = new double[6];

        p0 = new RevoluteJoint.Parameters();
        l[0] = 0.04;
        p0.lSegment = l[0];
        p0.rMin = -Math.PI;
        p0.rMax = Math.PI;
        p0.orientation = RevoluteJoint.Z_AXIS;

        p1 = new RevoluteJoint.Parameters();
        l[1] = 0.101;
        p1.lSegment = l[1];
        p1.rMin = Math.toRadians(-120.0);
        p1.rMax = Math.toRadians(120.0);
        p1.orientation = RevoluteJoint.Y_AXIS;

        p2 = new RevoluteJoint.Parameters();
        l[2] = 0.098;
        p2.lSegment = l[2];
        p2.rMin = Math.toRadians(-125.0);
        p2.rMax = Math.toRadians(125.0);
        p2.orientation = RevoluteJoint.Y_AXIS;

        p3 = new RevoluteJoint.Parameters();
        l[3] = 0.077;
        p3.lSegment = l[3];
        p3.rMin = Math.toRadians(-125.0);
        p3.rMax = Math.toRadians(125.0);
        p3.orientation = RevoluteJoint.Y_AXIS;

        p4 = new RevoluteJoint.Parameters();        // Wrist
        l[4] = 0.0;
        p4.lSegment = l[4];
        p4.rMin = Math.toRadians(-150.0);
        p4.rMax = Math.toRadians(150.0);
        p4.orientation = RevoluteJoint.Z_AXIS;

        p5 = new RevoluteJoint.Parameters();        // Hand joint, actually. Will need an upgrade
        l[5] = 0.101;
        p5.lSegment = l[5];
        p5.rMin = Math.toRadians(-40.0);
        p5.rMax = Math.toRadians(120.0);
        p5.orientation = RevoluteJoint.Y_AXIS;

        j0 = new RevoluteJoint(p0);
        j1 = new RevoluteJoint(p1);
        j2 = new RevoluteJoint(p2);
        j3 = new RevoluteJoint(p3);
        j4 = new RevoluteJoint(p4);
        j5 = new RevoluteJoint(p5);

        joints.add(j0);
        joints.add(j1);
        joints.add(j2);
        joints.add(j3);
        joints.add(j4);
        joints.add(j5);
    }

    // Handle ARM STATUS!
    public void messageReceived(LCM lcm, String channel, LCMDataInputStream ins)
    {
        try {
            messageReceivedEx(lcm, channel, ins);
        } catch (IOException ex) {
            System.out.println("ERR: "+ex);
        }
    }

    // XXX Do something about arm status
    public void messageReceivedEx(LCM lcm, String channel, LCMDataInputStream ins) throws IOException
    {
        if (channel.equals("ARM_STATUS")) {
            dynamixel_status_list_t statuses = new dynamixel_status_list_t(ins);
        }
    }

    class RenderThread extends Thread
    {
        int fps = 60;

        VisWorld vw;
        VisLayer vl;
        VisCanvas vc;

        public RenderThread()
        {
            JFrame jf = new JFrame("Bolt Arm Simulation Demo");
            jf.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
            jf.setLayout(new BorderLayout());
            jf.setSize(1920, 1080);

            vw = new VisWorld();
            vl = new VisLayer(vw);
            vc = new VisCanvas(vl);
            jf.add(vc, BorderLayout.CENTER);

            // Grid
            VzGrid.addGrid(vw);

            // Default zoom
            vl.cameraManager.fit2D(new double[] {-1,-1}, new double[] {1,1}, true);

            // Event handler
            vl.addEventHandler(new GoalEventHandler());

            jf.setVisible(true);
        }

        public void run()
        {
            while (true) {
                // Render Arm
                {
                    VisWorld.Buffer vb = vw.getBuffer("arm");
                    vb.addBack(new VisChain(LinAlg.rotateZ(-Math.PI/2),
                                            new VzTriangle(0.08, 0.08, 0.08,
                                                           new VzMesh.Style(Color.green))));
                    vb.addBack(new VisChain(LinAlg.translate(0,0,baseHeight/2),
                                            new VzBox(0.04, 0.04, baseHeight,
                                                      new VzMesh.Style(Color.black))));

                    double[][] xform = LinAlg.translate(0,0,baseHeight);
                    for (Joint j: joints) {
                        LinAlg.timesEquals(xform, j.getRotation());
                        vb.addBack(new VisChain(LinAlg.copy(xform),
                                                j.getVis()));
                        LinAlg.timesEquals(xform, j.getTranslation());
                    }
                    vb.swap();
                }

                // Draw current goal
                {
                    VisWorld.Buffer vb = vw.getBuffer("goal");
                    double[] xyz = kt.get();
                    if (xyz != null) {
                        vb.addBack(new VisChain(LinAlg.translate(xyz),
                                                new VzCircle(0.015, new VzMesh.Style(Color.yellow))));
                    }
                    vb.swap();
                }

                TimeUtil.sleep(1000/fps);
            }
        }
    }

    class KinematicsThread extends Thread
    {
        // Refresh rate
        int Hz = 100;

        // Current goal
        Object goalLock = new Object();
        double[] goal = null;
        double[] origin = new double[3];

        // Arm information and planning parameters
        double dh    = 0.08;        // Height of end effector above ground
        double minR  = 0.05;        // Min distance at which we plan
        double maxSR;               // Max distance at which we can plan gripper-down
        double maxCR;               // Max complex plan distance

        // LCM Stuff
        robot_command_t cmd;

        public KinematicsThread()
        {
            cmd = new robot_command_t();

            double h0 = l[0]+baseHeight;
            double l1 = l[1]+l[2];
            double q0 = l[3]+l[4]+l[5]+dh - h0;
            maxSR = Math.sqrt(l1*l1 - q0*q0);

            double q1;
            if (dh < h0) {
                q1 = dh;
            } else {
                q1 = dh - h0;
            }
            double l2 = l[3]+l[4]+l[5];
            double l3 = l1+l2;
            maxCR = Math.sqrt(l3*l3 - q1*q1)*.99; // XXX Hack

            System.out.printf("%f %f\n", maxSR, maxCR);
        }

        public void set(double[] goal_)
        {
            synchronized (goalLock) {
                goal = goal_;
                cmd.updateDest = true;
            }
        }

        public double[] get()
        {
            return goal;
        }

        public void run()
        {
            while (true) {
                synchronized (goalLock) {
                    // Compute distance of goal from origin. (assume z = 0)
                    // If this distance r is < maxR, use conservative
                    // planning. Otherwise, add last angle into the mix.
                    if (goal != null) {
                        double r = LinAlg.distance(origin, goal);
                        if (r < minR) {
                            // We don't plan this close to the arm
                        } else if (r < maxSR) {
                            simplePlan(r);
                        } else if (r < maxCR) {
                            complexPlan(r);
                        } else {
                            // Out of range
                            outOfRange(r);
                            //System.err.println("ERR: Goal outside of gripping range");
                        }
                        if (opts.getBoolean("lcm")) {
                            lcm.publish("ARM_COMMAND", getArmCommandList());
                        }
                        cmd.utime = TimeUtil.utime();
                        cmd.dest = LinAlg.resize(goal, 6);
                        cmd.action = "POINT";
                        lcm.publish("ROBOT_COMMAND", cmd);
                        cmd.updateDest = false;
                    }
                }
                TimeUtil.sleep(1000/Hz);
            }
        }

        private dynamixel_command_list_t getArmCommandList()
        {
            dynamixel_command_list_t cmds = new dynamixel_command_list_t();
            cmds.len = joints.size();
            cmds.commands = new dynamixel_command_t[cmds.len];
            long utime = TimeUtil.utime();
            for (int i = 0; i < joints.size(); i++) {
                dynamixel_command_t cmd = joints.get(i).getArmCommand();
                cmd.utime = utime;
                cmds.commands[i] = cmd;
            }
            return cmds;
        }

        // Plans with the wrist DOWN for ease of object grabbing
        private void simplePlan(double r)
        {
            double[] t = new double[6];
            t[0] = MathUtil.atan2(goal[1], goal[0]);

            double h = (l[3]+l[4]+l[5]+dh) - (l[0]+baseHeight);
            double lp = Math.sqrt(h*h + r*r);

            double l1_2 = l[1]*l[1];
            double l2_2 = l[2]*l[2];
            double lp_2 = lp*lp;

            double g0 = Math.acos((l1_2 + l2_2 - lp_2)/(2*l[1]*l[2]));
            double g1 = Math.acos((l1_2 + lp_2 - l2_2)/(2*l[1]*lp));
            double g2 = Math.acos((l2_2 + lp_2 - l1_2)/(2*l[2]*lp));
            double g3 = Math.acos(r/lp);
            double g4 = Math.acos(h/lp);

            t[1] = Math.PI/2 - g1 - g3;
            t[2] = Math.PI - g0;
            t[3] = Math.PI - g2 - g4;

            //t[5] = Math.toRadians(112.0);

            for (int i = 0; i < t.length; i++) {
                joints.get(i).set(t[i]);
            }
        }

        // Plans with wrist able to take different orientations
        private void complexPlan(double r)
        {
            double[] t = new double[6];
            t[0] = MathUtil.atan2(goal[1], goal[0]);
            double h = (l[0]+baseHeight) - dh;
            double lp = Math.sqrt(h*h + r*r);

            double l1 = l[1]+l[2];
            double l2 = l[3]+l[4]+l[5];

            double lp_2 = lp*lp;
            double l1_2 = l1*l1;
            double l2_2 = l2*l2;

            double g0 = Math.acos(h/lp);
            double g2 = Math.acos((l1_2 + l2_2 - lp_2)/(2*l1*l2));
            double g3 = Math.acos((l1_2 + lp_2 - l2_2)/(2*l1*lp));

            t[1] = Math.PI - g0 - g3;
            t[3] = Math.PI - g2;

            //t[5] = Math.toRadians(112.0);

            for (int i = 0; i < t.length; i++) {
                joints.get(i).set(t[i]);
            }
        }

        // Just point the arm towards the goal...Points a little low. XXX Controller?
        private void outOfRange(double r)
        {
            double[] t = new double[6];
            double tiltFactor = 20;
            t[0] = MathUtil.atan2(goal[1], goal[0]);
            t[1] = Math.PI/2 - Math.toRadians(tiltFactor);
            t[3] = Math.toRadians(tiltFactor);

            double l1 = l[0]+baseHeight;
            double l2 = l[1]+l[2];

            //t[3] = Math.min(0, MathUtil.atan2(l1, r-l2));
            //t[3] = MathUtil.atan2(l1, r-l2);

            //t[5] = Math.toRadians(112.0);

            for (int i = 0; i < t.length; i++) {
                joints.get(i).set(t[i]);
            }
        }
    }

    class GoalEventHandler extends VisEventAdapter
    {
        public int getDispatchOrder()
        {
            return 0;
        }

        public boolean mousePressed(VisCanvas vc, VisLayer vl, VisCanvas.RenderInfo rinfo, GRay3D ray, MouseEvent e)
        {
            double[] xyz = ray.intersectPlaneXY();
            int mods = e.getModifiersEx();
            boolean shift = (mods & MouseEvent.SHIFT_DOWN_MASK) > 0;
            boolean ctrl = (mods & MouseEvent.CTRL_DOWN_MASK) > 0;
            if (shift) {
                kt.set(xyz);
                return true;
            } else if (ctrl) {
                kt.set(null);
                return true;
            }

            return false;
        }

        public boolean mouseDragged(VisCanvas vc, VisLayer vl, VisCanvas.RenderInfo rinfo, GRay3D ray, MouseEvent e)
        {
            double[] xyz = ray.intersectPlaneXY();
            int mods = e.getModifiersEx();
            boolean shift = (mods & MouseEvent.SHIFT_DOWN_MASK) > 0;
            boolean ctrl = (mods & MouseEvent.CTRL_DOWN_MASK) > 0;
            if (shift) {
                kt.set(xyz);
                return true;
            } else if (ctrl) {
                kt.set(null);
                return true;
            }

            return false;
        }
    }

    static public void main(String[] args)
    {
        GetOpt opts = new GetOpt();
        opts.addBoolean('h',"help",false,"Display this help screen");
        opts.addBoolean('l',"lcm",false,"Emit LCM");

        if (!opts.parse(args)) {
            System.err.println("ERR: Option error - "+opts.getReason());
            return;
        }

        if (opts.getBoolean("help")) {
            opts.doHelp();
            return;
        }

        BoltArmDemo bd = new BoltArmDemo(opts);
    }
}
