package abolt.arm;

import java.io.*;
import java.util.*;
import java.awt.*;
import javax.swing.*;

import lcm.lcm.*;

import april.jmat.*;
import april.util.*;
import april.vis.*;

//import abolt.kinect.*;
import abolt.lcmtypes.*;
import abolt.util.*;

// XXX
import kinect.kinect.Segment;
import kinect.kinect.ObjectInfo;
import kinect.kinect.KUtils;

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

    // Debugging
    boolean debug;
    DebugThread dthread;

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

    class DebugThread extends Thread
    {
        VisWorld vw;
        public void run()
        {
            System.out.println("Starting debugging thread");
            vw = new VisWorld();
            VisLayer vl = new VisLayer(vw);
            VisCanvas vc = new VisCanvas(vl);

            vl.cameraManager.fit2D(new double[] {-1,-1}, new double[] {1,1}, true);

            VzGrid grid = VzGrid.addGrid(vw);

            JFrame jf = new JFrame("Bolt Arm Command Interpreter Debugger");
            jf.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
            jf.setLayout(new BorderLayout());
            jf.setSize(800, 600);

            jf.add(vc, BorderLayout.CENTER);

            jf.setVisible(true);
        }

        public void render(ArrayList<double[]> points)
        {
            System.out.println("Render object");
            ArrayList<double[]> flat = flattenPoints(k2wPointAlign(points));

            double[] cxy = getMeanXY(flat);
            // Render the XY centroid
            {
                VisWorld.Buffer vb = vw.getBuffer("centroid");
                vb.addBack(new VzPoints(new VisVertexData(cxy),
                                        new VzPoints.Style(Color.white, 4)));
                vb.swap();
            }

            double theta = getMinimalRotation(flat);
            ArrayList<double[]> rorigin = rotateAtOrigin(theta, flat);

            // Render the flattened points
            {
                VisWorld.Buffer vb = vw.getBuffer("points");
                vb.addBack(new VzPoints(new VisVertexData(flat),
                                        new VzPoints.Style(Color.red, 2)));
                vb.swap();
            }

            double[][] evec = getGripAxes(rorigin);//get22EigenVectors(rorigin);
            evec = rotateGripAxes(-theta, evec);
            // Render the major and minor axis
            {
                ArrayList<double[]> major = new ArrayList<double[]>();
                ArrayList<double[]> minor = new ArrayList<double[]>();
                major.add(new double[2]);
                major.add(evec[0]);
                minor.add(new double[2]);
                minor.add(evec[1]);
                VisWorld.Buffer vb = vw.getBuffer("eigen");
                vb.addBack(new VisChain(LinAlg.translate(cxy),
                                        new VzLines(new VisVertexData(major),
                                                    VzLines.LINES,
                                                    new VzLines.Style(Color.yellow, 2))));
                vb.addBack(new VisChain(LinAlg.translate(cxy),
                                        new VzLines(new VisVertexData(minor),
                                                    VzLines.LINES,
                                                    new VzLines.Style(Color.cyan, 2))));
                vb.swap();
            }


        }
    }

    public BoltArmCommandInterpreter(Segment seg_)
    {
        this(seg_, false);
    }

    public BoltArmCommandInterpreter(Segment seg_, boolean debug_)
    {
        debug = debug_;
        if (debug) {
            dthread = new DebugThread();
            dthread.start();
        }

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
        bcmd.obj_id = 0;

        // Check for a specified ID
        String objIDstr = SimUtil.getTokenValue(cmd.action, "POINT");  // XXX ID? POINT?

        if (objIDstr == null) {
            // No ID
            bcmd.xyz = LinAlg.resize(cmd.dest, 3);
        } else {
            // Found ID
            int objID = Integer.valueOf(objIDstr);
            if (debug) {
                System.out.println("POINT @ "+objID);
            }
            bcmd.obj_id = objID;

            ObjectInfo info = getObject(objID);
            if (info == null) {
                bcmd.xyz = LinAlg.resize(cmd.dest, 3);
            } else {
                if (debug) {
                    dthread.render(info.points);
                }
                ArrayList<double[]> points = flattenPoints(k2wPointAlign(info.points));
                bcmd.xyz = getCentroidXYZ(points);
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
        bcmd.obj_id = 0;

        // Check for a specified ID
        String objIDstr = SimUtil.getTokenValue(cmd.action, "GRAB"); // XXX ID? GRAB?
        if (objIDstr == null) {
            return null;    // There is no safe way to grab nothing
        } else {
            // Found ID
            int objID = Integer.valueOf(objIDstr);
            if (debug) {
                System.out.println("GRAB @ "+objID);
            }
            bcmd.obj_id = objID;
            ObjectInfo info = getObject(objID);
            if (info == null) {
                return null;    // There is no safe way to grab nothing
            } else {
                if (debug) {
                    dthread.render(info.points);
                }
                ArrayList<double[]> wPoints = k2wPointAlign(info.points);
                ArrayList<double[]> xyPoints = flattenPoints(wPoints);
                double[] uxy = getMeanXY(xyPoints);
                bcmd.xyz = LinAlg.resize(uxy, 3);

                double minBoxRot = getMinimalRotation(xyPoints);
                ArrayList<double[]> rorigin = rotateAtOrigin(minBoxRot, xyPoints);
                double[][] ev = getGripAxes(rorigin);//get22EigenVectors(rorigin);
                ev = rotateGripAxes(-minBoxRot, ev);

                double[] xaxis = new double[] {1.0, 0, 0};
                double[] a = LinAlg.normalize(uxy);
                double[] b = LinAlg.normalize(ev[1]);
                double[] c = LinAlg.normalize(ev[0]);
                //System.out.printf("[%f %f] . [%f %f]\n", a[0], a[1], b[0], b[1]);

                // Wrist action. Based on quadrants. Rotate to counter arm rotation?
                double theta = Math.acos(b[0]);

                double[] cross = LinAlg.crossProduct(xaxis, LinAlg.resize(c, 3));
                if (cross[2] > 0) {
                    theta = theta - Math.PI/2;
                } else {
                    theta = Math.PI/2 - theta;
                }

                // If in the positive Y quadrant, rotate angle 180 degrees
                if (a[1] > 0.0) {
                    theta += Math.PI;
                }

                // Account for arm rotation
                theta += Math.atan2(a[1], a[0]);

                // Mod the angle...
                theta = clampAngle(theta);

                bcmd.wrist = theta;
            }
        }

        return bcmd;
    }

    private double clampAngle(double theta)
    {
        System.out.printf("theta: [%f] ",theta);
        theta = MathUtil.mod2pi(theta);
        System.out.printf("mod2pi: [%f] ",theta);
        double max = Math.toRadians(150.0);
        double min = Math.toRadians(-150.0);
        if (theta > 0 && theta > max) {
            theta = MathUtil.mod2pi(theta - Math.PI);
            System.out.printf(" big: [%f]\n", theta);
            return theta;
        } else if (theta < 0 && theta < min) {
            theta = MathUtil.mod2pi(theta + Math.PI);
            System.out.printf(" small: [%f]\n", theta);
            return theta;
        }
        System.out.printf("\n");
        return theta;
    }

    /** Instruct the arm to drop an object at the specified location */
    private bolt_arm_command_t processDropCommand(robot_command_t cmd)
    {
        bolt_arm_command_t bcmd = new bolt_arm_command_t();
        bcmd.cmd_id = messageID++;
        bcmd.action = "DROP";

        bcmd.obj_id = 0; // XXX - not true
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
        bcmd.obj_id = 0;

        return bcmd;
    }

    // ==========================
    /** Align the points with the world frame */
    private ArrayList<double[]> k2wPointAlign(ArrayList<double[]> points)
    {
        ArrayList<double[]> wPoints = new ArrayList<double[]>();
        for (double[] p: points) {
            double[] w = KUtils.getWorldCoordinates(p);
            wPoints.add(w);
            //System.out.printf("[%f %f %f] == [%f %f %f]\n", p[0], p[1], p[2], w[0], w[1], w[2]);
        }

        return wPoints;
    }

    /** Flatten the supplied points onto the XY plane, trying to preserve only
     *  the upper shape of the object
     */
    private ArrayList<double[]> flattenPoints(ArrayList<double[]> points)
    {
        ArrayList<Integer> idxs = new ArrayList<Integer>();
        idxs.add(0);
        idxs.add(1);
        double r = 0.0025;
        return Binner.binPoints(points, idxs, r);
    }

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

        return xyz;
    }

    /** Find the mean XY position of the pixels */
    private double[] getMeanXY(ArrayList<double[]> points)
    {
        double[] xy = new double[2];
        double frac = 1.0/points.size();
        for (int i = 0; i < points.size(); i++) {
            double[] p = points.get(i);
            xy[0] += p[0]*frac;
            xy[1] += p[1]*frac;
        }

        return xy;
    }

    private double[][] getCovXY(ArrayList<double[]> points)
    {
        if (points.size() < 1) {
            System.err.println("ERR: Inssuficient points to compute covariance");
            return null;
        }

        double[] uxy = getMeanXY(points);
        //System.out.printf("mean: [%f %f]\n", uxy[0], uxy[1]);
        double[][] B = new double[2][points.size()];
        double[][] Bt = new double[points.size()][2];

        int i = 0;
        for (double[] p: points) {
            B[0][i] = p[0] - uxy[0];
            B[1][i] = p[1] - uxy[1];
            Bt[i][0] = B[0][i];
            Bt[i][1] = B[1][i];

            i++;
        }

        double[][] cov = LinAlg.matrixAB(B, Bt);
        cov = LinAlg.scale(cov, 1.0/points.size());
        //LinAlg.print(cov);
        return cov;
    }

    /** Find the angle of rotation around Z that minimizes the area
     *  of the axis-aligned bounding box. */
    private double getMinimalRotation(ArrayList<double[]> points)
    {
        // Transform points to origin.
        double[] cxy = getMeanXY(points);
        ArrayList<double[]> centered = LinAlg.transform(LinAlg.translate(-cxy[0], -cxy[1]),
                                                        points);

        // for every rotation 0-180 degrees, compute the bounding box.
        // Pick the minimal bounding box and rotate the points to that orientation and
        // return them.
        double minArea = Double.MAX_VALUE;
        double angle = 0;
        for (double i = 0.0; i < 180.0; i += 0.25) {
            ArrayList<double[]> rotate = LinAlg.transform(LinAlg.rotateZ(Math.toRadians(i)),
                                                          centered);
            double minX = Double.MAX_VALUE;
            double maxX = Double.MIN_VALUE;
            double minY = Double.MAX_VALUE;
            double maxY = Double.MIN_VALUE;

            for (double[] p: rotate) {
                minX = Math.min(minX, p[0]);
                maxX = Math.max(maxX, p[0]);
                minY = Math.min(minY, p[1]);
                maxY = Math.max(maxY, p[1]);
            }

            double area = (maxX - minX) * (maxY - minY);
            if (area < minArea) {
                minArea = area;
                angle = Math.toRadians(i);
            }
        }

        return angle;
    }

    /** Rotate the points around their centroid and move them to the origin */
    private ArrayList<double[]> rotateAtOrigin(double theta, ArrayList<double[]> points)
    {
        double[] cxy = getMeanXY(points);
        ArrayList<double[]> centered = LinAlg.transform(LinAlg.translate(-cxy[0], -cxy[1]),
                                                        points);

        ArrayList<double[]> rotated = LinAlg.transform(LinAlg.rotateZ(theta),
                                                       centered);
        return rotated;
    }

    /** Rotate the points around their centroid by the given angle */
    private ArrayList<double[]> rotateInPlace(double theta, ArrayList<double[]> points)
    {
        double[] cxy = getMeanXY(points);
        return LinAlg.transform(LinAlg.translate(cxy), rotateAtOrigin(theta, points));
    }

    /** Rotate the vectors such that they remain pointing towards "positive" */
    private double[][] rotateGripAxes(double theta, double[][] axes)
    {
        double[] ax0 = LinAlg.transform(LinAlg.rotateZ(theta), axes[0]);
        double[] ax1 = LinAlg.transform(LinAlg.rotateZ(theta), axes[1]);

        if (ax1[0] < 0) {
            ax1[0] = -ax1[0];
            ax1[1] = -ax1[1];
        }

        if (ax0[0] < 0) {
            ax0[0] = -ax0[0];
            ax0[1] = -ax0[1];
        }

        if (debug) {
            System.out.printf("[%f %f], [%f %f]\n", ax0[0], ax0[1], ax1[0], ax1[1]);
        }

        return new double[][] {ax0, ax1};
    }

    /** Return the axis-aligned vectors in the major and minor directions */
    private double[][] getGripAxes(ArrayList<double[]> points)
    {
        double minX = Double.MAX_VALUE;
        double maxX = Double.MIN_VALUE;
        double minY = Double.MAX_VALUE;
        double maxY = Double.MIN_VALUE;

        for (double[] p: points) {
            minX = Math.min(minX, p[0]);
            maxX = Math.max(maxX, p[0]);
            minY = Math.min(minY, p[1]);
            maxY = Math.max(maxY, p[1]);
        }

        if (maxX - minX > maxY - minY) {
            return new double[][] {{1.0, 0}, {0, 1.0}};
        } else {
            return new double[][] {{0, 1.0}, {1.0, 0}};
        }
    }

    /** Return eigenvectors along the major and minor axes of the points */
    private double[][] get22EigenVectors(ArrayList<double[]> points)
    {
        double[][] A = getCovXY(points);

        double a = A[0][0];
        double b = A[0][1];
        double c = A[1][0];
        double d = A[1][1];

        double tr = a + d;
        double det = a*d - b*c;

        // Eigenvalues
        double l0 = tr/2 + Math.sqrt(tr*tr/4 - det);
        double l1 = tr/2 - Math.sqrt(tr*tr/4 - det);

        // Compute Eigenvectors
        double[] e0 = new double[2];
        double[] e1 = new double[2];
        if (b != 0) {
            e0[0] = 1.0;
            e0[1] = e0[0]*(l0 - a)/b;

            e1[0] = 1.0;
            e1[1] = e1[0]*(l1 - a)/b;
        } else if (c != 0) {
            e0[1] = 1.0;
            e0[0] = e0[1]*(l0 - d)/c;

            e1[1] = 1.0;
            e1[0] = e1[1]*(l1 - d)/c;
        } else {
            e0[0] = 1.0;
            e0[1] = 0.0;
            e1[0] = 0.0;
            e1[1] = 1.0;
        }
        e0 = LinAlg.normalize(e0);
        e1 = LinAlg.normalize(e1);

        // Return "sorted" list of eigenvectors where first
        // vector returned is the major axis of the object
        // and the second is the minor
        double[][] evec = new double[2][2];
        evec[0][0] = e0[0];
        evec[0][1] = e0[1];
        evec[1][0] = e1[0];
        evec[1][1] = e1[1];

        if (debug) {
            System.out.printf("[%f %f], [%f %f]\n", e0[0], e0[1], e1[0], e1[1]);
        }

        return evec;
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
