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
    double baseHeight   = BoltArm.baseHeight;
    double goalHeight   = 0.08;     // Goal height of end effector
    double transHeight  = 0.20;     // Transition height of end effector
    double minR         = 0.05;     // Minimum distance away from arm center at which we plan
    double maxGSR;                  // Max distance at which we can plan gripper-down (goalHeight)
    double maxGCR;                  // Max distance at which we can actually plan (goalHeight);
    double maxTSR;                  // Max distance at which we can plan gripper-down (transHeight)
    double maxTCR;                  // Max distance at which we can actually plan (transHeight);

    // Controller stability stuff
    long holdTime = 500000;         // Time to hold position [micro s] until transitioning to next state
    int minHistory = 10;            // Minimum number of position history to keep
    double stableError = 0.01;      // If position is always within this much for our window, stable

    class PositionTracker
    {
        LinkedList<Pair<double[], Long> > positions = new LinkedList<Pair<double[], Long> >();

        public void clear()
        {
            positions.clear();
        }

        public void add(double[] xyz, long utime) {
            positions.add(new Pair<double[], Long>(xyz, utime));
        }

        public double error()
        {
            long utime = TimeUtil.utime();

            while (positions.size() > minHistory) {
                Pair<double[], Long> front = positions.getFirst();
                if ((utime - front.getSecond()) > holdTime) {
                    positions.pop();
                } else {
                    break;
                }
            }

            if (positions.size() < minHistory) {
                return Double.MAX_VALUE;
            }

            double[] avgXYZ = new double[3];
            for (Pair<double[], Long> p: positions) {
                avgXYZ = LinAlg.add(avgXYZ, p.getFirst());
            }
            for (int i = 0; i < avgXYZ.length; i++) {
                avgXYZ[i] /= positions.size();
            }

            double err = 0;
            for (Pair<double[], Long> p: positions) {
                err += LinAlg.magnitude(LinAlg.subtract(avgXYZ, p.getFirst()));
            }

            return err/positions.size();
        }

    }

    // A simple elbow-up controller
    class ControlThread extends Thread
    {
        // Update rate
        int Hz = 100;

        // Current target position
        int state = 0;
        robot_command_t last_cmd = null;
        double[] prev = null;
        double[] goal = null;
        PositionTracker ptracker;

        // Last issued command for controller
        dynamixel_command_list_t last_cmds;


        public ControlThread()
        {
            ptracker = new PositionTracker();
        }

        public void run()
        {
            while (true) {
                TimeUtil.sleep(1000/Hz);
                // States:
                //      0: New command, transition to UP in current pos
                //      1: Reached UP successfully, transition to UP in new pos
                //      2: Reached UP_NEW successfully, transition to DOWN in new pos

                // Establish whether or not to reset to state 0:
                robot_command_t cmd = cmds.get();
                if (cmd != null && cmd.updateDest && (last_cmd == null || last_cmd.utime < cmd.utime)) {
                    last_cmd = cmd;
                    prev = goal;
                    goal = LinAlg.resize(cmd.dest, 2);
                    setState(0);
                }

                if (goal == null)
                    continue;

                // Compute arm end-effector position from status messages
                dynamixel_status_list_t dsl = statuses.get();
                if (dsl != null) {
                    double[][] xform = LinAlg.translate(0,0,BoltArm.baseHeight);
                    for (int i = 0; i < dsl.len; i++) {
                        if (i == 0) {
                            LinAlg.timesEquals(xform, LinAlg.rotateZ(dsl.statuses[i].position_radians));
                        } else if (i+1 < dsl.len) {
                            LinAlg.timesEquals(xform, LinAlg.rotateY(dsl.statuses[i].position_radians));
                        }
                        LinAlg.timesEquals(xform, joints.get(i).getTranslation());
                    }
                    double[] currXYZ = LinAlg.resize(LinAlg.matrixToXyzrpy(xform), 3);
                    ptracker.add(currXYZ, TimeUtil.utime());
                }

                double r = LinAlg.magnitude(goal);
                double error = ptracker.error();
                if (state == 0) {
                    if (prev == null) {
                        setState(state+1);
                        continue;
                    }

                    // Try to move arm up above prev
                    if (r < minR) {
                        // Don't do anythign here. Too close to the arm
                    } else if (r < maxTSR) {
                        simplePlan(r, prev, transHeight);
                    } else if (r < maxTCR) {
                        complexPlan(r, prev, transHeight);
                    } else {
                        setState(state+1);
                    }

                    // Check to see if it's time to transition
                    if (error < stableError) {
                        setState(state+1);
                    }

                } else if (state == 1) {
                    if (prev == null) {
                        setState(state+1);
                        continue;
                    }

                    // Try to move arm up above goal
                    if (r < minR) {
                        // Don't do anythign here. Too close to the arm
                    } else if (r < maxTSR) {
                        simplePlan(r, goal, transHeight);
                    } else if (r < maxTCR) {
                        complexPlan(r, goal, transHeight);
                    } else {
                        setState(state+1);
                    }

                    // Check to see if it's time to transition
                    if (error < stableError) {
                        setState(state+1);
                    }
                } else if (state == 2) {
                    if (r < minR) {
                        // Don't do anything here. Too close to the arm
                    } else if (r < maxGSR) {
                        simplePlan(r, goal, goalHeight);
                    } else if (r < maxGCR) {
                        complexPlan(r, goal, goalHeight);
                    } else {
                        outOfRange(r, goal);
                    }
                }

                dynamixel_command_list_t desired_cmds = getArmCommandList();
                lcm.publish("ARM_COMMAND", desired_cmds);
            }
        }

        void setState(int s)
        {
            state = s;
            ptracker.clear();
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
        private void simplePlan(double r, double[] goal, double height)
        {
            double[] t = new double[6];
            t[0] = MathUtil.atan2(goal[1], goal[0]);

            double h = (l[3]+l[4]+l[5]+height) - (l[0]+baseHeight);
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

            //t[5] = Math.toRadians(112.0); XXX

            for (int i = 0; i < t.length; i++) {
                joints.get(i).set(t[i]);
            }
        }

        // Plans with wrist able to take different orientations
        private void complexPlan(double r, double[] goal, double height)
        {
            double[] t = new double[6];
            t[0] = MathUtil.atan2(goal[1], goal[0]);
            double h = (l[0]+baseHeight) - height;
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

            //t[5] = Math.toRadians(112.0); XXX

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

            //t[5] = Math.toRadians(112.0); XXX

            for (int i = 0; i < t.length; i++) {
                joints.get(i).set(t[i]);
            }
        }
    }

    public BoltArmController()
    {
        initArm();

        // Initialize controller properties (goalHeight)
        double h0 = l[0]+baseHeight;
        double l1 = l[1]+l[2];
        double q0 = l[3]+l[4]+l[5]+goalHeight - h0;
        maxGSR = Math.sqrt(l1*l1 - q0*q0);

        double q1;
        if (goalHeight < h0) {
            q1 = goalHeight;
        } else {
            q1 = goalHeight - h0;
        }
        double l2 = l[3]+l[4]+l[5];
        double l3 = l1+l2;
        maxGCR = Math.sqrt(l3*l3 - q1*q1)*.99; // XXX Hack

        // Initialize controller properties (transHeight)
        q0 = l[3]+l[4]+l[5]+transHeight + h0;
        maxTSR = Math.sqrt(l1*l1 - q0*q0);

        if (transHeight < h0) {
            q1 = transHeight;
        } else {
            q1 = transHeight - h0;
        }
        maxTCR = Math.sqrt(l3*l3 - q1*q1)*.99; // XXX Hack

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
        joints = BoltArm.initArm();
        l = new double[6];
        int i = 0;
        for (Joint j: joints) {
            l[i++] = ((RevoluteJoint)j).getLength();    //  XXX Will break when we add hand
        }
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
    // === Get rendering information ===
    public ArrayList<Joint> getJoints()
    {
        return joints;
    }

    // ==================================================
    static public void main(String[] args)
    {
        BoltArmController bac = new BoltArmController();
    }
}
