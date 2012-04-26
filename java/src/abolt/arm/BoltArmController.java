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

    // Update rate
    int Hz = 100;

    // Arm parameters and restrictions
    ArrayList<Joint> joints;
    double[] l;
    double baseHeight   = BoltArm.baseHeight;

    class PositionTracker
    {
        LinkedList<Pair<double[], Long> > positions = new LinkedList<Pair<double[], Long> >();
        long holdTime = 500000; // Time to hold position [micro s] until transitioning to next state
        int minHistory = (int)(Hz*(holdTime/1000000.0));

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

            //System.out.println(err/positions.size());
            return err/positions.size();
        }

    }

    // A simple elbow-up controller
    class ControlThread extends Thread
    {
        // Current target position
        int state = 0;
        robot_command_t last_cmd = null;
        double[] prev = null;
        double[] goal = null;
        PositionTracker ptracker;

        // Controller stability stuff
        double stableError = 0.0005;              // If position is always within this much for our window, "stable"

        // previous command/status state
        dynamixel_command_list_t last_cmds;
        dynamixel_status_list_t last_status;

        // General planning
        double minR         = 0.05;     // Minimum distance away from arm center at which we plan

        // Pointing
        double goalHeight   = 0.08;     // Goal height of end effector
        double transHeight  = 0.20;     // Transition height of end effector

        // Grabbing & sweeping
        double sweepHeight  = 0.025;    // Height at which we sweep objects along
        double grabHeight   = 0.020;    // Height at which we try to grab objects

        public ControlThread()
        {
            ptracker = new PositionTracker();
        }

        public void run()
        {
            while (true) {
                TimeUtil.sleep(1000/Hz);

                // Compute arm end-effector position over time from status messages
                dynamixel_status_list_t dsl = statuses.get();
                if (dsl != null) {
                    double[][] xform = LinAlg.translate(0,0,BoltArm.baseHeight);
                    for (int i = 0; i < dsl.len; i++) {
                        if (i == 0) {
                            LinAlg.timesEquals(xform, LinAlg.rotateZ(dsl.statuses[i].position_radians));
                        } else if (i+2 < dsl.len) {
                            LinAlg.timesEquals(xform, LinAlg.rotateY(dsl.statuses[i].position_radians));
                        }
                        LinAlg.timesEquals(xform, joints.get(i).getTranslation());
                    }
                    double[] currXYZ = LinAlg.resize(LinAlg.matrixToXyzrpy(xform), 3);
                    //System.out.printf("[%f %f %f]\n", currXYZ[0], currXYZ[1], currXYZ[2]);
                    ptracker.add(currXYZ, TimeUtil.utime());
                }

                // Action handler
                // If a new action comes in, reset to beginning of appropriate state machine
                // Otherwise, continue executing current state machine
                robot_command_t cmd = cmds.get();

                // Check for new action
                if (cmd != null && (last_cmd == null || last_cmd.utime < cmd.utime)) {
                    last_cmd = cmd;
                    prev = goal;
                    goal = LinAlg.resize(cmd.dest, 2);
                    setState(0);
                }

                if (last_cmd != null) {
                    if (last_cmd.action.contains("POINT")) {
                        pointStateMachine();
                    } else if (last_cmd.action.contains("GRAB")) {
                        grabStateMachine();
                    } else if (last_cmd.action.contains("DROP")) {
                        dropStateMachine();
                    } else if (last_cmd.action.contains("SWEEP")) {
                        sweepStateMachine();
                    } else if (last_cmd.action.contains("RESET")) {
                        resetArm();
                    }
                }
                last_status = dsl;

                dynamixel_command_list_t desired_cmds = getArmCommandList();
                lcm.publish("ARM_COMMAND", desired_cmds);
            }
        }

        void setState(int s)
        {
            assert (last_cmd != null);
            System.out.printf("%s: [%d]\n", last_cmd.action, s);
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

        /** Move the arm to point at the currently specified goal */
        private void pointStateMachine()
        {
            // States:
            //      0: New command, transition to UP in current pos
            //      1: Reached UP successfully, transition to UP in new pos
            //      2: Reached UP_NEW successfully, transition to DOWN in new pos
            double r = LinAlg.magnitude(goal);
            double error = ptracker.error();
            if (state == 0) {
                if (prev == null) {
                    setState(state+1);
                    return;
                }

                moveTo(prev, transHeight);

                // Check to see if it's time to transition
                if (error < stableError) {
                    setState(state+1);
                }

            } else if (state == 1) {
                moveTo(goal, transHeight);

                // Check to see if it's time to transition
                if (error < stableError) {
                    setState(state+1);
                }
            } else if (state == 2) {
                moveTo(goal, goalHeight);

                if (error < stableError) {
                    setState(state+1);
                }
            }
        }

        /** Eventually, try to grab the object at the specified point.
         *  For now, just try to swing the arm through it to change the
         *  view point
         */
        private void sweepStateMachine()
        {
            double r = LinAlg.magnitude(goal);
            double error = ptracker.error();

            double angle = Math.atan2(goal[1], goal[0]);
            double negAngle = angle-Math.toRadians(30);
            double posAngle = angle+Math.toRadians(30);
            double newAngle = MathUtil.mod2pi(negAngle);
            if (newAngle != negAngle) {
                newAngle = MathUtil.mod2pi(posAngle);
            }

            // States
            //      XXX: Go to transition height above current position
            //      0: Go to a position -30 degrees behind desired up high
            //      1: Move to correct height
            //      2: Ram the arm into the object at position "GOAL"
            //      3: Move back up
            if (state == 0) {
                if (prev == null) {
                    setState(state+1);
                    return;
                }

                moveTo(prev, transHeight);

                if (error < stableError) {
                    setState(state+1);
                }
            } else if (state == 1) {
                moveTo(goal, transHeight);

                RevoluteJoint j = (RevoluteJoint)joints.get(0);
                j.set(newAngle);

                if (error < stableError) {
                    setState(state+1);
                }
            } else if (state == 2) {
                moveTo(goal, sweepHeight, sweepHeight*2);

                RevoluteJoint j = (RevoluteJoint)joints.get(0);
                j.set(newAngle);

                if (error < stableError) {
                    setState(state+1);
                }
            } else if (state == 3) {
                moveTo(goal, sweepHeight, sweepHeight*2);

                if (error < stableError) {
                    setState(state+1);
                }
            } else if (state == 3) {
                moveTo(goal, sweepHeight);
            }
        }

        /** Actual grab state machine. Moves to location, centering arm with gripper to
         *  the side of goal slightly. XXX For now, assumes that block is properly aligned
         *  for gripping. Moves above, then down, then slowly closes on object until we
         *  are confident that we are gripping to object OR we have fully closed the hand.
         *  Then we await further orders.
         */
        private void grabStateMachine()
        {
            double r = LinAlg.magnitude(goal);
            double error = ptracker.error();

            // Compute "safe" goal just to side of grab point
            double offset = 0.02;
            double angle = Math.atan2(goal[1], goal[0]);
            double dtheta = Math.asin(offset/r);
            double newangle = MathUtil.mod2pi(angle+dtheta);
            double minSpeed = HandJoint.HAND_SPEED/2.0;

            // States
            //      0: move to high point at current position
            //      1: move above point (slightly to side) at a safe height
            //      2: move down to grabbing height
            //      3: Start closing hand
            //      4: Check for contact with an object/hand stopping
            //      5: Adjust grip so we grab tightly, but don't break hand
            if (state == 0) {
                // Open hand
                joints.get(5).set(0.0);

                if (prev == null) {
                    setState(state+1);
                    return;
                }

                moveTo(prev, transHeight);

                if (error < stableError) {
                    setState(state+1);
                }
            } else if (state == 1) {
                moveTo(goal, transHeight);

                RevoluteJoint j = (RevoluteJoint)(joints.get(0));
                j.set(newangle);

                if (error < stableError) {
                    setState(state+1);
                }
            } else if (state == 2) {
                moveTo(goal, grabHeight, grabHeight*2); // XXX can't grab close objects :(

                RevoluteJoint j = (RevoluteJoint)(joints.get(0));
                j.set(newangle);

                if (error < stableError) {
                    setState(state+1);
                }
            } else if (state == 3) {
                HandJoint j = (HandJoint)(joints.get(5));
                dynamixel_status_list_t dsl = statuses.get();
                if (dsl == null) {
                    j.set(0.0);
                    return;
                }
                dynamixel_status_t gripper_status = dsl.statuses[5];

                // Start the hand closing
                j.set(112.0);
                if (gripper_status.speed > minSpeed) {
                    setState(state+1);
                }
            } else if (state == 4) {
                // Check for hand contact with semi-rigid object (we'll stop moving)
                HandJoint j = (HandJoint)(joints.get(5));
                dynamixel_status_list_t dsl = statuses.get();
                if (dsl == null) {
                    j.set(0.0);
                    return;
                }

                dynamixel_status_t gripper_status = dsl.statuses[5];
                double speed = Math.abs(gripper_status.speed);

                if (speed < minSpeed) {
                    j.set(gripper_status.position_radians);
                    setState(state+1);
                }
            } else if (state == 5) {
                // Tweak grip strength so we don't break the hand
                HandJoint j = (HandJoint)(joints.get(5));
                dynamixel_status_list_t dsl = statuses.get();
                if (dsl == null) {
                    j.set(0.0);
                    return;
                }

                dynamixel_status_t gripper_status = dsl.statuses[5];
                double maxLoad = 0.35;
                double minLoad = 0.20;
                double gripIncr = Math.toRadians(3.0);
                double load = Math.abs(gripper_status.load);
                System.out.printf("[%f] <= [%f (%f)] < [%f]\n", minLoad, load, gripper_status.load, maxLoad);

                if (gripper_status.load <= 0.0 && load < minLoad) {
                    //System.out.println("Grip moar");
                    j.set(gripper_status.position_radians + gripIncr);
                } else if (gripper_status.load <= 0.0 && load > maxLoad) {
                    //System.out.println("Grip less");
                    j.set(gripper_status.position_radians - gripIncr);
                }
            }
        }

        /** Drop the object we are currently holding at the goal*/
        private void dropStateMachine()
        {
            double r = LinAlg.magnitude(goal);
            double error = ptracker.error();

            // States
            //      0: Up to transition height
            //      1: Over to drop point
            //      2: Down to drop height
            //      3: Release

            if (state < 3) {
                pointStateMachine();
            } else {
                HandJoint j = (HandJoint)(joints.get(5));
                j.set(0.0);
            }
        }

        // ======================================================

        private void resetArm()
        {
            for (Joint j: joints) {
                j.set(0.0);
            }
        }

        private void moveTo(double[] goal, double height)
        {
            moveTo(goal, height, height);
        }

        /** Use one of the various planners to move to the specified goal */
        private void moveTo(double[] goal, double heightS, double heightC)
        {
            // Compute gripping ranges on the fly
            double minR = 0.05;
            double maxSR;
            double maxCR;

            // Initialize controller properties for height
            double h0 = l[0]+baseHeight;
            double l1 = l[1]+l[2];
            double q0 = l[3]+l[4]+l[5]+heightS - h0;
            maxSR = Math.sqrt(l1*l1 - q0*q0);

            double q1;
            if (heightC < h0) {
                q1 = heightC;
            } else {
                q1 = heightC - h0;
            }
            double l2 = l[3]+l[4]+l[5];
            double l3 = l1+l2;
            maxCR = Math.sqrt(l3*l3 - q1*q1)*.90; // XXX Hack

            double r = LinAlg.magnitude(LinAlg.resize(goal, 2));

            // Collision check: make sure we haven't gotten stopped by something
            // before reaching our goal XXX

            if (r < minR) {
                // Do nothing, we can't plan at this range
            } else if (r < maxSR) {
                simplePlan(r, goal, heightS);
            } else if (r < maxCR) {
                complexPlan(r, goal, heightC);
            } else {
                outOfRange(r, goal);
            }
        }

        // Plans with the wrist DOWN for ease of object grabbing
        private void simplePlan(double r, double[] goal, double height)
        {
            double[] t = new double[5];
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
            double[] t = new double[5];
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
            double[] t = new double[5];
            double tiltFactor = 45;
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
            if (j instanceof RevoluteJoint) {
                l[i++] = ((RevoluteJoint)j).getLength();
            } else if (j instanceof HandJoint) {
                l[i++] = ((HandJoint)j).getLength();
            }
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
