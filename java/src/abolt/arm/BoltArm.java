package abolt.arm;

import java.util.*;
import java.io.*;
import java.awt.*;

import lcm.lcm.*;

import april.util.*;
import april.jmat.*;
import april.vis.*;

import abolt.lcmtypes.*;

/** Global inteface to the physical bolt arm. Contains the joints
 *  for the arm as well as the current status. Objects interested
 *  in the state of the arm can make requests to the BoltArm,
 *  which will update the accordingly.
 */
public class BoltArm implements LCMSubscriber
{
    private LCM lcm = LCM.getSingleton();

    final static double baseHeight = 0.075;
    private ArrayList<Joint> joints;

    private ExpiringMessageCache<dynamixel_status_list_t> statuses = new ExpiringMessageCache<dynamixel_status_list_t>(0.2, true);

    private BoltArm()
    {
        initArm();

        lcm.subscribe("ARM_STATUS", this);
    }

    private static BoltArm singleton = null;
    synchronized public static BoltArm getSingleton()
    {
        if (singleton == null) {
            singleton = new BoltArm();
        }

        return singleton;
    }

    /** Create a model of the arm. Change segment
     *  dimensions and constraints on movement here
     */
    private void initArm()
    {
        joints = new ArrayList<Joint>();
        Joint j0, j1, j2, j3, j4, j5;
        RevoluteJoint.Parameters p0, p1, p2, p3, p4;

        p0 = new RevoluteJoint.Parameters();
        p0.lSegment = 0.04;
        p0.rMin = -Math.PI;
        p0.rMax = Math.PI;
        p0.orientation = RevoluteJoint.Z_AXIS;

        p1 = new RevoluteJoint.Parameters();
        p1.lSegment = 0.101;
        p1.rMin = Math.toRadians(-120.0);
        p1.rMax = Math.toRadians(120.0);
        p1.orientation = RevoluteJoint.Y_AXIS;

        p2 = new RevoluteJoint.Parameters();
        p2.lSegment = 0.098;
        p2.rMin = Math.toRadians(-125.0);
        p2.rMax = Math.toRadians(125.0);
        p2.orientation = RevoluteJoint.Y_AXIS;

        p3 = new RevoluteJoint.Parameters();
        p3.lSegment = 0.077;
        p3.rMin = Math.toRadians(-125.0);
        p3.rMax = Math.toRadians(125.0);
        p3.orientation = RevoluteJoint.Y_AXIS;

        p4 = new RevoluteJoint.Parameters();        // Wrist
        p4.lSegment = 0.0;
        p4.rMin = Math.toRadians(-150.0);
        p4.rMax = Math.toRadians(150.0);
        p4.orientation = RevoluteJoint.Z_AXIS;

        j0 = new RevoluteJoint(p0);
        j1 = new RevoluteJoint(p1);
        j2 = new RevoluteJoint(p2);
        j3 = new RevoluteJoint(p3);
        j4 = new RevoluteJoint(p4);
        j5 = new HandJoint(new HandJoint.Parameters());

        joints.add(j0);
        joints.add(j1);
        joints.add(j2);
        joints.add(j3);
        joints.add(j4);
        joints.add(j5);
    }

    public void messageReceived(LCM lcm, String channel, LCMDataInputStream ins)
    {
        try {
            messageReceivedEx(lcm, channel, ins);
        } catch (IOException ioex) {
            System.err.println("ERR: LCM channel "+channel);
            ioex.printStackTrace();
        }
    }

    public void messageReceivedEx(LCM lcm, String channel, LCMDataInputStream ins) throws IOException
    {
        if (channel.equals("ARM_STATUS")) {
            // Handle arm status. Updates the joint internal state
            // as well as saves the most recently sent status message
            // for consumption by the user
            dynamixel_status_list_t dsl = new dynamixel_status_list_t(ins);
            long utime = Long.MAX_VALUE;
            for (dynamixel_status_t s: dsl.statuses) {
                utime = Math.min(utime, s.utime);
            }
            if (statuses.put(dsl, utime)) {
                for (int i = 0; i < dsl.len; i++) {
                    joints.get(i).updatePos(dsl.statuses[i].position_radians);
                }
            }
        }
    }

    // === User interface to arm =======

    /** Return a list of joints in the arm, base to hand */
    public ArrayList<Joint> getJoints()
    {
        return joints;
    }

    /** Get the position of the gripper */
    public double[] getGripperXYZRPY()
    {
        double[][] xform = LinAlg.translate(0,0,baseHeight);
        for (Joint j: joints) {
            LinAlg.timesEquals(xform, j.getRotation());
            LinAlg.timesEquals(xform, j.getTranslation());
        }

        return LinAlg.matrixToXyzrpy(xform);
    }

    /** Returns the most recent status of the requested servo */
    public dynamixel_status_t getStatus(int idx)
    {
        dynamixel_status_list_t dsl = statuses.get();
        if (dsl == null)
            return null;
        if (idx >= dsl.len)
            return null;
        return dsl.statuses[idx];
    }

    /** Returns the desired position of the requested servo */
    public double getDesiredPos(int idx)
    {
        if (idx >= joints.size())
            return Double.MAX_VALUE;
        return joints.get(idx).getDesiredValue();
    }

    /** Returns the actual position of the requested servo */
    public double getActualPos(int idx)
    {
        if (idx >= joints.size())
            return Double.MAX_VALUE;
        return joints.get(idx).getActualValue();
    }

    /** Get the length of segment idx */
    public double getLength(int idx)
    {
        if (idx >= joints.size())
            return -1;
        return joints.get(idx).getLength();
    }

    public double getMinValue(int idx)
    {
        if (idx >= joints.size())
            return Double.MAX_VALUE;
        return joints.get(idx).getMinValue();
    }

    public double getMaxValue(int idx)
    {
        if (idx >= joints.size())
            return Double.MAX_VALUE;
        return joints.get(idx).getMaxValue();
    }

    public dynamixel_command_t getCommand(int idx)
    {
        if (idx >= joints.size())
            return null;
        return joints.get(idx).getArmCommand();
    }

    /** Set the position of servo idx to pos */
    public void setPos(int idx, double pos)
    {
        if (idx >= joints.size())
            return;
        joints.get(idx).setPos(pos);
    }

    /** Render the arm in the supplied vis world */
    public void render(VisWorld vw)
    {
        VisWorld.Buffer vb = vw.getBuffer("arm");

        // Render a base (XXX Update me)
        vb.addBack(new VisChain(LinAlg.rotateZ(-Math.PI/2),
                                new VzTriangle(0.08, 0.08, 0.08,
                                               new VzMesh.Style(Color.green))));
        vb.addBack(new VisChain(LinAlg.translate(0, 0, baseHeight/2),
                                new VzBox(0.04, 0.04, baseHeight,
                                          new VzMesh.Style(Color.black))));

        // Render the joints
        double[][] xform = LinAlg.translate(0, 0, baseHeight);
        for (Joint j: joints) {
            LinAlg.timesEquals(xform, j.getRotation());
            vb.addBack(new VisChain(LinAlg.copy(xform),
                                    j.getVis()));
            LinAlg.timesEquals(xform, j.getTranslation());
        }
        vb.swap();
    }
}
