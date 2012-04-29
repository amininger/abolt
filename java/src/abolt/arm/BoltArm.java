package abolt.arm;

import java.util.*;

// Just a convenience class for intializing an arm
public class BoltArm
{
    final static public double baseHeight = 0.075;

    public static ArrayList<Joint> initArm()
    {
        ArrayList<Joint> joints = new ArrayList<Joint>();
        Joint j0, j1, j2, j3, j4, j5;
        RevoluteJoint.Parameters p0, p1, p2, p3, p4, p5;

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

        //p5 = new RevoluteJoint.Parameters();        // Hand joint, actually. Will need an upgrade
        //p5.lSegment = 0.101;
        //p5.rMin = Math.toRadians(-40.0);
        //p5.rMax = Math.toRadians(120.0);
        //p5.orientation = RevoluteJoint.Y_AXIS;

        j0 = new RevoluteJoint(p0);
        j1 = new RevoluteJoint(p1);
        j2 = new RevoluteJoint(p2);
        j3 = new RevoluteJoint(p3);
        j4 = new RevoluteJoint(p4);
        //j5 = new RevoluteJoint(p5);
        j5 = new HandJoint(new HandJoint.Parameters());

        joints.add(j0);
        joints.add(j1);
        joints.add(j2);
        joints.add(j3);
        joints.add(j4);
        joints.add(j5);

        return joints;
    }

}
