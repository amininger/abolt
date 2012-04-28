package abolt.arm;

import java.io.*;
import java.util.*;

import lcm.lcm.*;

import april.jmat.*;
import april.util.*;

import abolt.kinect.*;
import abolt.lcmtypes.*;
import abolt.util.*;

// XXX
import kinect.kinect.Segment;
import kinect.kinect.ObjectInfo;

/** The command interpreter takes in a robot_command_t from
 *  Soar and then processes it for consumption by the
 *  BoltArmController. Output should be bolt_arm_command_t
 *  or something of that order.
 */
public class BoltArmCommandInterpreter implements LCMSubscriber
{
    // LCM
    LCM lcm = LCM.getSingleton();
    static private byte messageID = 0;

    // Don't hold on to old messages
    ExpiringMessageCache<robot_command_t> cmds = new ExpiringMessageCache<robot_command_t>(20.0, true);

    // Needs some form of access to point cloud data + IDs
    Segment seg;

    class InterpreterThread extends Thread
    {
        // Rate at which we look for new commands as
        // well as broadcast commands to the controller
        int Hz = 5;

        boolean handled = false;
        robot_command_t last_cmd = null;
        bolt_arm_command_t bolt_cmd = null;

        public void run()
        {
            while (true) {
                TimeUtil.sleep(1000/Hz);

                // Look for new commands
                robot_command_t cmd = cmds.get();
                if ((cmd != null) &&
                    (last_cmd == null || last_cmd.utime < cmd.utime))
                {
                    last_cmd = cmd;
                    handled = false;
                }

                // Process new commands
                if (last_cmd != null && !handled) {
                    System.out.println(last_cmd.action);
                    if (last_cmd.action.contains("POINT")) {
                        bolt_cmd = processPointCommand(last_cmd);
                    } else if (last_cmd.action.contains("GRAB")) {
                        bolt_cmd = processGrabCommand(last_cmd);
                    } else if (last_cmd.action.contains("DROP")) {
                        bolt_cmd = processDropCommand(last_cmd);
                    } else if (last_cmd.action.contains("RESET")) {
                        bolt_cmd = processResetCommand(last_cmd);
                    } else {
                        System.err.println("ERR: Unknown command - "+last_cmd.action);
                    }
                    handled = true;
                }

                // Broadcast command message to robot
                if (bolt_cmd != null) {
                    bolt_cmd.utime = TimeUtil.utime();
                    lcm.publish("BOLT_ARM_COMMAND", bolt_cmd);
                }
            }
        }
    }

    public BoltArmCommandInterpreter(Segment seg_)
    {
        // We'll reference this, or some equivalent, later when
        // recovering point cloud data
        seg = seg_;

        // Soar sends us robot commands through this channel, ordering
        // us to "POINT", "GRAB", and "DROP" objects.
        lcm.subscribe("ROBOT_COMMAND", this);

        // Thread waits for new commands to arrive, processing them and
        // sending them on to the arm, having now grounded the command
        // in the real world. Thus, a POINT=ID command will find the
        // object associated with ID, extract its location from the point
        // cloud data, and send a command to the arm to point to that
        // location.
        InterpreterThread interpreter = new InterpreterThread();
        interpreter.start();
    }

    public void messageReceived(LCM lcm, String channel, LCMDataInputStream ins)
    {
        try {
            messageReceivedEx(lcm, channel, ins);
        } catch (IOException ioex) {
            System.err.println("ERR: LCM");
            ioex.printStackTrace();
        }
    }

    public void messageReceivedEx(LCM lcm, String channel, LCMDataInputStream ins) throws IOException
    {
        if (channel.equals("ROBOT_COMMAND")) {
            System.out.println("GOT ROBOT COMMAND");
            robot_command_t cmd = new robot_command_t(ins);
            cmds.put(cmd, cmd.utime);
        }
    }

    // === Command processing ===
    /** Instruct the arm to point at a specific location OR, if an object
     *  ID is specified, at that specific object. Lacking an object matching
     *  that ID, point at the location in XYZRPY.
     */
    private bolt_arm_command_t processPointCommand(robot_command_t cmd)
    {
        bolt_arm_command_t bcmd = new bolt_arm_command_t();
        bcmd.cmd_id = messageID++;
        bcmd.action = "POINT";
        bcmd.wrist = 0.0;         // We don't care about the wrist

        // Check for a specified ID
        String objIDstr = SimUtil.getTokenValue(cmd.action, "POINT");  // XXX ID? POINT?

        if (objIDstr == null) {
            // No ID
            bcmd.xyz = LinAlg.resize(cmd.dest, 3);
        } else {
            // Found ID
            int objID = Integer.valueOf(objIDstr);
            System.out.println("POINT @ "+objID);
            ObjectInfo info = getObject(objID);
            System.out.println("Found info "+info);
            if (info == null) {
                bcmd.xyz = LinAlg.resize(cmd.dest, 3);
            } else {
                bcmd.xyz = getCentroidXYZ(info.points);
            }
        }

        return bcmd;
    }

    /** Instruct the arm to grab the object specified by ID */
    private bolt_arm_command_t processGrabCommand(robot_command_t cmd)
    {
        bolt_arm_command_t bcmd = new bolt_arm_command_t();
        bcmd.cmd_id = messageID++;
        bcmd.action = "GRAB";


        // Check for a specified ID
        String objIDstr = SimUtil.getTokenValue(cmd.action, "GRAB"); // XXX ID? GRAB?
        if (objIDstr == null) {
            return null;    // There is no safe way to grab nothing
        } else {
            // Found ID
            int objID = Integer.valueOf(objIDstr);
            System.out.println("GRAB @ "+objID);
            ObjectInfo info = getObject(objID);
            System.out.println("Found info "+info);
            if (info == null) {
                return null;    // There is no safe way to grab nothing
            } else {
                bcmd.xyz = getCentroidXYZ(info.points);

                // Wrist action XXX
                bcmd.wrist = 0;
            }
        }

        return bcmd;
    }

    /** Instruct the arm to drop an object at the specified location */
    private bolt_arm_command_t processDropCommand(robot_command_t cmd)
    {
        bolt_arm_command_t bcmd = new bolt_arm_command_t();
        bcmd.cmd_id = messageID++;
        bcmd.action = "DROP";

        bcmd.xyz = LinAlg.resize(cmd.dest, 3);
        bcmd.wrist = 0; // XXX

        return bcmd;
    }

    /** Instuct the arm to reset to its default position */
    private bolt_arm_command_t processResetCommand(robot_command_t cmd)
    {
        bolt_arm_command_t bcmd = new bolt_arm_command_t();
        bcmd.cmd_id = messageID++;
        bcmd.action = "RESET";
        bcmd.xyz = new double[3];
        bcmd.wrist = 0;

        return bcmd;
    }

    // ==========================
    /** Return the centroid of the given point cloud. Used for
     *  pointing and for gripping.
     */
    private double[] getCentroidXYZ(ArrayList<double[]> points)
    {
        double[] xyz = new double[3];
        if (points == null || points.size() < 1)
            return xyz;

        double frac = 1.0/points.size();
        for (int i = 0; i < points.size(); i++) {
            double[] p = points.get(i);
            xyz[0] += p[0]*frac;
            xyz[1] += p[1]*frac;
            xyz[2] += p[2]*frac;
        }
        double[] kxyz = kinect.kinect.KUtils.getWorldCoordinates(xyz);
        System.out.printf("[%f %f %f] -- [%f %f %f]\n", xyz[0], xyz[1], xyz[2],
                                                        kxyz[0], kxyz[1], kxyz[2]);

        // Hope we loaded up XXX
        return kxyz;
    }

    /** Return the ObjectInfo for a relevant ID */
    private ObjectInfo getObject(int id)
    {
        for (Integer key: seg.objects.keySet()) {
            ObjectInfo info = seg.objects.get(key);
            if (info.repID == id)
                return info;
        }
        return null;
    }



    // ==========================

    static public void main(String[] args)
    {
        BoltArmCommandInterpreter baci = new BoltArmCommandInterpreter(null);
    }
}
