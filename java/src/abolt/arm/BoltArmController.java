package abolt.arm;

import java.util.*;
import java.io.*;

import lcm.lcm.*;

import april.jmat.*;
import april.util.*;

import abolt.lcmtypes.*;
import abolt.util.*;

/** Receives as input commands from user interface. These
 *  high level commands are converted into scripted arm
 *  movements and sent to the ArmDriver for execution.
 */
public class BoltArmController implements LCMSubscriber
{
    // LCM
    LCM lcm = LCM.getSingleton();
    Queue<bolt_arm_command_t> cmds = new LinkedList<bolt_arm_command_t>();

    // Update rate
    int Hz = 25;

    // Arm parameters and restrictions
    BoltArm arm = BoltArm.getSingleton();
    //ArrayList<Joint> joints;
    double[] l;
    double baseHeight   = BoltArm.baseHeight;

    // State Tracking
    enum ActionState
    {
        POINT_UP_CURR, POINT_UP_OVER, POINT_AT, POINT_WAITING,
        GRAB_UP_CURR, GRAB_UP_BEHIND, GRAB_APPROACH,
            GRAB_AT, GRAB_START_GRIP, GRAB_GRIPPING,
            GRAB_ADJUST_GRIP, GRAB_WAITING,
        DROP_UP_CURR, DROP_UP_OVER, DROP_AT, DROP_RELEASE,
            DROP_RETREAT, DROP_WAITING,
        HOME, HOMING,
    }
    private ActionState state = ActionState.HOME;

    // Track the current mode for LCM messages
    enum ActionMode
    {
        WAIT, HOME, GRAB, POINT, DROP, FAILURE
    }
    private ActionMode curAction = ActionMode.WAIT;
    private int grabbedObject = -1;
    private int toGrab = -1;

    public int grabbedObject(){
    	if(grabbedObject != -1){
    		return grabbedObject;
    	} else {
    		return toGrab;
    	}
    }
    public static BoltArmController Singleton = null;

    // A simple elbow-up controller
    class ControlThread extends Thread
    {
        // Current target position
        boolean newAction = false;
        bolt_arm_command_t last_cmd = null;
        double[] prev = null;
        double[] goal = null;

        // Controller stability stuff
        long actionStartTime = 0;           // Time the last action was started in [uSec]
        long actionCompleteTime = 500000;   // Time to complete action in [uSec]

        // previous command/status state
        dynamixel_command_list_t last_cmds;

        // Pointing
        double goalHeight   = 0.0;      // Goal height of end effector
        double preGrabOffset = 0.03;
        double transOffset  = 0.20;     // Height above goal height to transition at

        // Defaults
        double defGrip      = Math.toRadians(30.0); // Keep the hand partially closed when possible

        public ControlThread()
        {
        }

        public void run()
        {
            while (true) {
                TimeUtil.sleep(1000/Hz);

                // Action handler - get the most recent command
                bolt_arm_command_t cmd = cmds.peek();

                // Check for new action. Shouldn't execute new
                // action until we have completed the previous one
                // (or entered a failure state)
                if (cmd != null &&
                    (last_cmd == null || last_cmd.cmd_id != cmd.cmd_id))
                {
                    last_cmd = cmd;
                    prev = goal;
                    goalHeight = cmd.xyz[2];    // XXX
                    goal = LinAlg.resize(cmd.xyz, 2);
                    //setState(0);
                    newAction = true;
                    if(cmd.action.contains("GRAB")){
                        toGrab = cmd.obj_id;
                        curAction = ActionMode.GRAB;
                    } else if (cmd.action.contains("POINT")) {
                        curAction = ActionMode.POINT;
                    } else if (cmd.action.contains("DROP")) {
                        curAction = ActionMode.DROP;
                    } else if (cmd.action.contains("RESET") ||
                               cmd.action.contains("HOME")) {
                        curAction = ActionMode.HOME;
                    }
                }

                if (last_cmd != null) {
                    if (last_cmd.action.contains("POINT")) {
                        pointStateMachine();
                    } else if (last_cmd.action.contains("GRAB")) {
                        grabStateMachine();
                    } else if (last_cmd.action.contains("DROP")) {
                        dropStateMachine();
                    } else if (last_cmd.action.contains("RESET")) {
                        if (newAction) {
                            resetArm();
                            grabbedObject = -1;
                            toGrab = -1;
                            setState(ActionState.HOMING);
                        } else if (actionComplete() && !state.equals(ActionState.HOME)) {
                            setState(ActionState.HOME);
                            curAction = ActionMode.WAIT;
                        }
                    } else if (last_cmd.action.contains("HOME")) {
                        if (newAction) {
                            homeArm();
                            curAction = ActionMode.HOME;
                            setState(ActionState.HOMING);

                        } else if (actionComplete() && !state.equals(ActionState.HOME)) {
                            setState(ActionState.HOME);
                            curAction = ActionMode.WAIT;
                        }
                    }
                }
                newAction = false;

                // If we are no longer acting, remove that command
                if (curAction == ActionMode.WAIT ||
                    curAction == ActionMode.FAILURE)
                {
                    cmds.poll();
                }

                dynamixel_command_list_t desired_cmds = getArmCommandList();
                lcm.publish("ARM_COMMAND", desired_cmds);
                robot_action_t current_action = getCurrentAction();
                double[] xyzrpy = arm.getGripperXYZRPY(); // Lauren (next three lines)
                if(current_action.action.equals("DROP"))
                   current_action.drop = new double[]{goal[0], goal[1], 0};
                current_action.xyz = new double[]{xyzrpy[0], xyzrpy[1], xyzrpy[2]};
                lcm.publish("ROBOT_ACTION", current_action);
            }
        }

        /** Set the state and reset the internal clock counter */
        void setState(ActionState s)
        {
            assert (last_cmd != null);
            // XXX DEBUG
            System.err.printf("%s: [%s]\n", last_cmd.action, s.toString());
            state = s;

            actionStartTime = TimeUtil.utime();
        }

        /** Returns true when the arm completes its current action. */
        private boolean actionComplete()
        {
            long timePast = TimeUtil.utime() - actionStartTime;

            // Must wait at least as long as our action timer specifies
            if (timePast < actionCompleteTime) {
                return false;
            }

            // XXX Need a num joints thing or something
            // Iterate through the joints and see if they're all moving or not
            for (int i = 0; i < arm.getJoints().size(); i++) {
                dynamixel_status_t stat = arm.getStatus(i);
                if (stat != null && !BoltMath.equals(0, stat.speed, 0.01)) {
                	// actionStartTime = TimeUtil.utime();
                    return false;
                }
            }

            return true;
        }

        // XXX Gross arm interface stuff
        private dynamixel_command_list_t getArmCommandList()
        {
            ArrayList<Joint> joints = arm.getJoints();
            dynamixel_command_list_t acmds = new dynamixel_command_list_t();
            acmds.len = joints.size();
            acmds.commands = new dynamixel_command_t[acmds.len];
            long utime = TimeUtil.utime();
            for (int i = 0; i < joints.size(); i++) {
                dynamixel_command_t cmd = arm.getCommand(i);
                cmd.utime = utime;
                acmds.commands[i] = cmd;
            }
            return acmds;
        }

        private robot_action_t getCurrentAction()
        {
            robot_action_t action = new robot_action_t();
            action.utime = TimeUtil.utime();
            action.obj_id = grabbedObject;
            action.action = curAction.toString();
            return action;
        }

        /** Move the arm to point at the currently specified goal */
        private void pointStateMachine()
        {
            // One time start up cost to set up the action
            if (newAction) {
                switch (state) {
                    case HOME:
                    case GRAB_WAITING:
                    case DROP_WAITING:
                        setState(ActionState.POINT_UP_OVER);
                        break;
                    case POINT_WAITING:
                        setState(ActionState.POINT_UP_CURR);
                        break;
                }
            }

            // States:
            //      0: New command, transition to UP in current pos
            //      1: Reached UP successfully, transition to UP in new pos
            //      2: Reached UP_NEW successfully, transition to DOWN in new pos
            //      3: Change state back to waiting
            double r = LinAlg.magnitude(goal);

            // XXX TODO: Better pointing grip

            switch (state) {
                case POINT_UP_CURR:
                    if (prev == null) {
                        setState(ActionState.POINT_UP_OVER);
                    }
                    moveTo(prev, goalHeight + transOffset);

                    if (actionComplete()) {
                        setState(ActionState.POINT_UP_OVER);
                    }
                    break;
                case POINT_UP_OVER:
                    moveTo(goal, goalHeight + transOffset);

                    if (actionComplete()) {
                        setState(ActionState.POINT_AT);
                    }
                    break;
                case POINT_AT:
                    moveTo(goal, goalHeight);

                    if (actionComplete()) {
                        setState(ActionState.POINT_WAITING);
                    }
                    break;
                case POINT_WAITING:
                    curAction = ActionMode.WAIT;
                    break;
                default:
                    // XXX DEBUG
                    System.err.println("ERR: Point state "+state.toString()+" does not exist");
                    break;
            }
        }

        /** Actual grab state machine. Moves to location, centering arm with gripper to
         *  the side of goal slightly. Moves above, then down, then slowly closes on object until we
         *  are confident that we are gripping to object OR we have fully closed the hand.
         *  Then we await further orders.
         */
        private void grabStateMachine()
        {
            // One time cost to start up action
            if (newAction) {
                switch (state) {
                    case HOME:
                    case GRAB_WAITING:
                    case DROP_WAITING:
                        setState(ActionState.GRAB_UP_BEHIND);
                        break;
                    case POINT_WAITING:
                        setState(ActionState.GRAB_UP_CURR);
                        break;
                }
            }

            double r = LinAlg.magnitude(goal);

            double angle = Math.atan2(goal[1], goal[0]);
            double minSpeed = HandJoint.HAND_SPEED/4.0;

            // Compute a point "behind" our object in gripper space
            double[] wrVec = new double[] {0.0, 1.0};
            wrVec = LinAlg.transform(LinAlg.rotateZ(angle), wrVec); // Base rotation
            wrVec = LinAlg.transform(LinAlg.rotateZ(-last_cmd.wrist), wrVec);
            wrVec = LinAlg.normalize(wrVec);
            double offset = 0.03;   // meters
            double[] behind = new double[] {goal[0] + (offset*wrVec[0]),
                                            goal[1] + (offset*wrVec[1])};

            // States
            //      0: move to high point at current position
            //      1: move behind point (slightly to side) at a safe height
            //      2: move closer behind point
            //      3: move down to grabbing height
            //      4: Start closing hand
            //      5: Check for contact with an object/hand stopping
            //      6: Adjust grip so we grab tightly, but don't break hand
            //      7: Switch action state back to waiting
            dynamixel_status_t gripper_status;
            double maxLoad = 0.400;
            double minLoad = 0.275;
            double gripIncr = Math.toRadians(2.5);
            switch (state) {
                case GRAB_UP_CURR:
                    // Open hand
                    arm.setPos(5, defGrip);

                    if (prev == null) {
                        setState(ActionState.GRAB_UP_BEHIND);
                        break;
                    }

                    moveTo(prev, goalHeight + transOffset);

                    if (actionComplete()) {
                        setState(ActionState.GRAB_UP_BEHIND);
                    }
                    break;
                case GRAB_UP_BEHIND:
                    // Open hand
                    arm.setPos(5, defGrip);

                    moveTo(behind, goalHeight + transOffset);

                    if (actionComplete()) {
                        setState(ActionState.GRAB_APPROACH);
                    }
                    if(toGrab > -1)
                        grabbedObject = toGrab; // Lauren
                    break;
                case GRAB_APPROACH:
                    moveTo(behind, goalHeight + preGrabOffset);

                    if (actionComplete()) {
                        setState(ActionState.GRAB_AT);
                    }
                    break;
                case GRAB_AT:
                    //moveTo(goal, grabHeight, grabHeight*1.8);   // Try to compensate for sagging arm
                    moveTo(goal, goalHeight);

                    if (actionComplete()) {
                        setState(ActionState.GRAB_START_GRIP);
                    }
                    break;
                case GRAB_START_GRIP:
                    gripper_status = arm.getStatus(5);
                    if (gripper_status == null) {
                        arm.setPos(5, defGrip);
                        break;
                    }

                    // Start the hand closing
                    arm.setPos(5, 112.0);
                    if (actionComplete()) {
                        setState(ActionState.GRAB_GRIPPING);
                    }
                    break;
                case GRAB_GRIPPING:
                    // Check for hand contact with semi-rigid object (we'll stop moving)
                    // Stop the gripping process at this point and start adjusting the
                    // strength of the grip
                    gripper_status = arm.getStatus(5);
                    if (gripper_status == null) {
                        arm.setPos(5, defGrip);
                        return;
                    }

                    double speed = Math.abs(gripper_status.speed);
                    if (speed < minSpeed) {
                        arm.setPos(5, gripper_status.position_radians);
                        setState(ActionState.GRAB_ADJUST_GRIP);
                    }

                    break;
                case GRAB_ADJUST_GRIP:
                    // Tweak the grip strength so we don't break the hand XXX
                    gripper_status = arm.getStatus(5);
                    if (gripper_status == null) {
                        arm.setPos(5, defGrip);
                        return;
                    }

                    double load = Math.abs(gripper_status.load);
                    //System.out.printf("[%f] <= [%f (%f)] < [%f]\n", minLoad, load, gripper_status.load, maxLoad);

                    if (actionComplete()) {
                        // At this point, we should have a solid grip.
                        // Go to the home position
                        setState(ActionState.HOME);
                    } else if (gripper_status.load <= 0.0 && load < minLoad) {
                        //System.out.println("Grip moar");
                        arm.setPos(5, gripper_status.position_radians + gripIncr);
                    } else if (gripper_status.load <= 0.0 && load > maxLoad) {
                        //System.out.println("Grip less");
                        arm.setPos(5, gripper_status.position_radians - gripIncr);
                    }

                    break;
                case HOME:
                    homeArm();

                    if (actionComplete()) {
                        setState(ActionState.GRAB_WAITING);
                    }
                    break;
                case GRAB_WAITING:
                    if (toGrab < 0)
                        break;

                    gripper_status = arm.getStatus(5);
                    if (gripper_status != null) {
                        if (!BoltMath.equals(gripper_status.load, 0, 0.01)) {
                            grabbedObject = toGrab;
                            curAction = ActionMode.WAIT;
                            toGrab = -1;
                        } else {
                            grabbedObject = -1;
                            curAction = ActionMode.FAILURE;
                            toGrab = -1;
                        }
                    }

                    break;
                default:
                    // XXX DEBUG
                    System.err.println("ERR: Grab state "+state.toString()+" does not exist");
                    break;
            }
        }

        /** Drop the object we are currently holding at the goal*/
        private void dropStateMachine()
        {
            // One time check to see how to transition into the machine
            if (newAction) {
                switch (state) {
                    case HOME:
                    case GRAB_WAITING:
                    case DROP_WAITING:
                        setState(ActionState.DROP_UP_OVER);
                        break;
                    case POINT_WAITING:
                        setState(ActionState.DROP_UP_CURR);
                        break;
                }
            }

            double r = LinAlg.magnitude(goal);

            // States
            //      0: Up to transition height
            //      1: Over to drop point
            //      2: Down to drop height
            //      3: Release
            //      4: Change state back to waiting
            switch (state) {
                case DROP_UP_CURR:
                    if (prev == null) {
                        setState(ActionState.DROP_UP_OVER);
                    }
                    moveTo(prev, goalHeight + transOffset);

                    if (actionComplete()) {
                        setState(ActionState.DROP_UP_OVER);
                    }
                    break;
                case DROP_UP_OVER:
                    moveTo(goal, goalHeight + transOffset);

                    if (actionComplete()) {
                        setState(ActionState.DROP_AT);
                    }
                    break;
                case DROP_AT:
                    moveTo(goal, goalHeight);
                    //moveTo(goal, grabHeight,grabHeight*1.8);
                    if (actionComplete()) {
                        setState(ActionState.DROP_RELEASE);
                    }
                    break;
                case DROP_RELEASE:
                    // Open the gripper
                    arm.setPos(5, defGrip);

                    // Transition the state once the gripper is definitively open
                    if (actionComplete()) {
                        setState(ActionState.DROP_RETREAT);
                    }
                    break;
                case DROP_RETREAT:
                    moveTo(goal, goalHeight + transOffset);

                    if (actionComplete()) {
                        setState(ActionState.HOME);
                    }
                    break;
                case HOME:
                    homeArm();

                    if (actionComplete()) {
                        setState(ActionState.DROP_WAITING);
                    }
                    break;
                case DROP_WAITING:
                    grabbedObject = -1;
                    curAction = ActionMode.WAIT;
                    break;
                default:
                    // XXX DEBUG
                    System.err.println("ERR: Drop state "+state.toString()+" does not exist");
            }
        }

        // ======================================================

        private void homeArm()
        {
            // XXX Need better thing than direct joint access
            // Doesn't touch the gripper
            for (int i = 0; i < arm.getJoints().size()-1; i++) {
                arm.setPos(i, 0);
            }
        }

        private void resetArm()
        {
            for (int i = 0; i < arm.getJoints().size(); i++) {
                arm.setPos(i, 0);
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
            double minR = 0.010;
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

            for (int i = 0; i < 4; i++) {
                arm.setPos(i, t[i]);
            }
            arm.setPos(4, last_cmd.wrist);  // XXX Persistent wrist command?
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

            for (int i = 0; i < 4; i++) {
                arm.setPos(i, t[i]);
            }
            arm.setPos(4, last_cmd.wrist);  // XXX Persistent wrist command?
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

            for (int i = 0; i < 4; i++) {
                arm.setPos(i, t[i]);
            }
            arm.setPos(4, last_cmd.wrist);  // XXX Persistent wrist command?
        }
    }

    public BoltArmController()
    {
    	Singleton = this;
        initArm();

        // Start independent control thread.
        ControlThread ct = new ControlThread();
        ct.start();

        lcm.subscribe("BOLT_ARM_COMMAND", this);
    }

    /** Construct a series of revolute joints representing our current arm
     *  layout. TODO: Make an appropriate hand joint for the end instead of
     *  a stick.
     */
    private void initArm()
    {
        // XXX We shouldn't have to do this! All of this lives in the arm, now
        l = new double[arm.getJoints().size()];
        for (int i = 0; i < l.length; i++) {
            l[i] = arm.getLength(i);
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
        if (channel.equals("BOLT_ARM_COMMAND")) {
            //robot_command_t cmd = new robot_command_t(ins);
            bolt_arm_command_t cmd = new bolt_arm_command_t(ins);
            //cmds.put(cmd, cmd.utime);
            cmds.add(cmd);
        }
    }


    // ==================================================
    static public void main(String[] args)
    {
        // Set things up to launch our own arm driver, too?

        BoltArmController bac = new BoltArmController();
    }
}
