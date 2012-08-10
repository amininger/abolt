package abolt.arm;

import java.util.*;
import java.io.*;
import java.awt.*;

import lcm.lcm.*;

import april.util.*;
import april.jmat.*;
import april.vis.*;
import april.config.*;

import abolt.lcmtypes.*;

/** Global inteface to the physical bolt arm. Contains the joints
 *  for the arm as well as the current status. Objects interested
 *  in the state of the arm can make requests to the BoltArm,
 *  which will update the accordingly.
 */
public class BoltArm implements LCMSubscriber
{
    private LCM lcm = LCM.getSingleton();

    public static double baseHeight = 0.0;
    public static double wristHeight;
    private ArrayList<Joint> joints = new ArrayList<Joint>();
    private ArrayList<Double> armWidths = new ArrayList<Double>();

    private ExpiringMessageCache<dynamixel_status_list_t> statuses = new ExpiringMessageCache<dynamixel_status_list_t>(0.5, true);

    private BoltArm()
    {
        //initArm();
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

    /** Create a model of the arm. Starts us off with
     *  some default parameters.
     */
    public void initArm(Config config)
    {
        joints.clear();
        Joint j0, j1, j2, j3, j4, j5;
        RevoluteJoint.Parameters p0, p1, p2, p3, p4;

        String name = config.getString("arm.arm_version", null);
        assert (name != null);

        baseHeight = config.getDouble("arm."+name+".base_height", 0);
        wristHeight = config.getDouble("arm."+name+".wrist_height",0);

        for (int i = 0;; i++) {
            double[] range = config.getDoubles("arm."+name+".r"+i+".range", null);
            double length = config.getDouble("arm."+name+".r"+i+".length", 0);
            String axis = config.getString("arm."+name+".r"+i+".axis", null);
            double speed = config.getDouble("arm."+name+".r"+i+".speed", 0);
            double torque = config.getDouble("arm."+name+".r"+i+".torque", 0);
            double width = config.getDouble("arm."+name+".r"+i+".width", 0);
            if (range == null)
                break;

            RevoluteJoint.Parameters params = new RevoluteJoint.Parameters();
            params.lSegment = length;
            params.rMin = Math.toRadians(range[0]);
            params.rMax = Math.toRadians(range[1]);
            if (axis.equals("X")) {
                params.orientation = RevoluteJoint.X_AXIS;
            } else if (axis.equals("Y")) {
                params.orientation = RevoluteJoint.Y_AXIS;
            } else if (axis.equals("Z")) {
                params.orientation = RevoluteJoint.Z_AXIS;
            } else {
                System.err.println("ERR: Bad axis specification - "+axis);
                System.err.println("Defaulting to Y-axis rotation");
                params.orientation = RevoluteJoint.Y_AXIS;
            }

            params.speed = speed;
            params.torque = torque;

            joints.add(new RevoluteJoint(params));
            armWidths.add(width);
        }

        // XXX Old hardcoded values
        /*
        p0 = new RevoluteJoint.Parameters();
        //p0.lSegment = 0.04;
        p0.lSegment = 0.051;
        p0.rMin = -Math.PI;
        p0.rMax = Math.PI;
        p0.orientation = RevoluteJoint.Z_AXIS;

        p1 = new RevoluteJoint.Parameters();
        //p1.lSegment = 0.101;
        p1.lSegment = 0.2245;
        p1.rMin = Math.toRadians(-120.0);
        p1.rMax = Math.toRadians(120.0);
        p1.orientation = RevoluteJoint.Y_AXIS;

        p2 = new RevoluteJoint.Parameters();
        //p2.lSegment = 0.098;
        p2.lSegment = 0.2000;
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
        */
        joints.add(new HandJoint(new HandJoint.Parameters()));
        armWidths.add(.05);
        armWidths.add(.05);
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

    /** Return a list of the widths of each arm segment.**/
    public ArrayList<Double> getArmWidths()
    {
        return armWidths;
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

    /** Get the height of the base.**/
    public double getBaseHeight()
    {
        return baseHeight;
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
