package abolt.arm;

import java.util.*;
import java.io.*;

import lcm.lcm.*;

import april.jmat.*;
import april.util.*;

import abolt.lcmtypes.*;

/** Received messages from SOAR side and uses them to position the arm,
 *  grab objects, and generally interact with the environment. Converts
 *  incoming robot_command_t messages into appropriate dynamixel messages
 *  for positioning the arm.
 */
public class BoltArmController implements LCMSubscriber
{
    // LCM
    LCM lcm = LCM.getSingleton();
    ExpiringMessageCache<dynamixel_status_list_t> statuses = new ExpiringMessageCache<dynamixel_status_list_t>(0.2, true);
    ExpiringMessageCache<robot_command_t> cmds = new ExpiringMessageCache<robot_command_t>(0.2, true);

    // Arm parameters and restrictions
    ArrayList<Joint> joints;
    double[] l;
    double baseHeight   = 0.075;
    double dh           = 0.08;     // Goal height of end effector
    double minR         = 0.05;     // Minimum distance away from arm center at which we plan
    double maxSR;                   // Max distance at which we can plan gripper-down
    double maxCR;                   // Max distance at which we can actually plan

    // A simple elbow-up controller
    class ControlThread extends Thread
    {
        // Update rate
        int Hz = 100;

        // Current target position
        double[] goal = null;
        double[] origin = new double[2];

        // Last issued command for controller
        dynamixel_command_list_t last_cmds;

        public ControlThread()
        {

        }

        public void run()
        {
            while (true) {
                robot_command_t cmd = cmds.get();
                if (cmd != null && cmd.updateDest) {
                    goal = LinAlg.resize(cmd.dest, 2);
                }

                if (goal != null) {
                    double r = LinAlg.distance(origin, goal);
                    if (r < minR) {
                        // Don't do anything here. Too close to the arm
                    //} else if (r < maxSR) {
                        //simplePlan(r, goal);
                    //} else if (r < maxCR) {
                        //complexPlan(r, goal);
                    } else {
                        outOfRange(r, goal);
                    }
                    dynamixel_command_list_t desired_cmds = getArmCommandList();

                    lcm.publish("ARM_COMMAND", desired_cmds);

                    // Desire: if arm is too far away from the desired position,
                    // ramp up requested position to correct for the error
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
        private void simplePlan(double r, double[] goal)
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

            t[5] = Math.toRadians(112.0);

            for (int i = 0; i < t.length; i++) {
                joints.get(i).set(t[i]);
            }
        }

        // Plans with wrist able to take different orientations
        private void complexPlan(double r, double[] goal)
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

            t[5] = Math.toRadians(112.0);

            for (int i = 0; i < t.length; i++) {
                joints.get(i).set(t[i]);
            }
        }

        // Just point the arm towards the goal...Points a little low. XXX Controller?
        private void outOfRange(double r, double[] goal)
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

            t[5] = Math.toRadians(112.0);

            for (int i = 0; i < t.length; i++) {
                joints.get(i).set(t[i]);
            }
        }
    }

    public BoltArmController()
    {
        initArm();

        // Initialize controller properties
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

        // Start independent control thread.
        ControlThread ct = new ControlThread();
        ct.start();

        lcm.subscribe("ARM_STATUS", this);
        lcm.subscribe("ROBOT_COMMAND", this);
    }

    /** Construct a series of revolute joints representing our current arm
     *  layout. TODO: Make an appropriate hand joint for the end instead of
     *  a stick.
     */
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

    /** Handle incoming LCM messages */
    public void messageReceived(LCM lcm, String channel, LCMDataInputStream ins)
    {
        try {
            messageReceivedEx(lcm, channel, ins);
        } catch (IOException ex) {
            System.out.println("ERR: "+ex);
        }
    }

    /** Responds to messages such as arm statuses and robot commands. */
    public void messageReceivedEx(LCM lcm, String channel, LCMDataInputStream ins) throws IOException
    {
        if (channel.equals("ARM_STATUS")) {
            dynamixel_status_list_t dsl = new dynamixel_status_list_t(ins);
            long utime = Long.MAX_VALUE;
            for (dynamixel_status_t s: dsl.statuses) {
                utime = Math.min(utime, s.utime);
            }
            statuses.put(dsl, utime);
        } else if (channel.equals("ROBOT_COMMAND")) {
            robot_command_t cmd = new robot_command_t(ins);
            cmds.put(cmd, cmd.utime);
        }
    }

    // ==================================================
    static public void main(String[] args)
    {
        BoltArmController bac = new BoltArmController();
    }
}
