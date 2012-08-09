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
import abolt.bolt.*;

import abolt.kinect.Segment;
import abolt.kinect.ObjectInfo;
import abolt.kinect.KUtils;

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

    // Queue up messages as we receive them, assuming we'll get
    // only one of each
    Queue<robot_command_t> cmds = new LinkedList<robot_command_t>();

    // Debugging
    boolean debug;
    DebugThread dthread;

    // Height values for commands
    double defaultPointHeight = 0.08;   // Default height above object to point [m]
    double pointOffset = 0.06;  // How many [m] to point at above the target
    double grabOffset  = 0.03;  // Max # [m] to reach down through the object
    //double dropOffset  = 0.06;  // How many [m] above the table to drop objects
    double dropOffset  = 0.02;

    // State on held object
    double heldHeight = 0;

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
                robot_command_t cmd = cmds.poll();
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
                    } else if (last_cmd.action.contains("HOME")) {
                        bolt_cmd = processHomeCommand(last_cmd);
                    } else {
                        System.err.println("ERR: Unknown command - "+last_cmd.action);
                    }
                    handled = true;

                    // Broadcast command message to robot
                    if (bolt_cmd != null) {
                        bolt_cmd.utime = TimeUtil.utime();
                        lcm.publish("BOLT_ARM_COMMAND", bolt_cmd);
                    }
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
            //ArrayList<double[]> flat = flattenPoints(k2wPointAlign(points));
            ArrayList<double[]> alignedPoints = k2wPointAlign(points);

            //double[] cxy = getMeanXY(flat);
            double[] cxy = BoltUtil.getCentroidXY(alignedPoints);
            // Render the XY centroid
            {
                VisWorld.Buffer vb = vw.getBuffer("centroid");
                vb.addBack(new VzPoints(new VisVertexData(cxy),
                                        new VzPoints.Style(Color.white, 4)));
                vb.swap();
            }

            /*
            double theta = getMinimalRotation(flat);
            ArrayList<double[]> rorigin = rotateAtOrigin(theta, flat);
            */
            double theta = BoltUtil.getBBoxTheta(alignedPoints);
            ArrayList<double[]> flat = BoltUtil.isolateTopFace(alignedPoints);
            ArrayList<double[]> rorigin = BoltUtil.rotateAtOrigin(flat,
                                                                  theta);

            // Render the flattened points
            {
                VisWorld.Buffer vb = vw.getBuffer("points");
                vb.addBack(new VzPoints(new VisVertexData(flat),
                                        new VzPoints.Style(Color.red, 2)));
                vb.addBack(new VzPoints(new VisVertexData(alignedPoints),
                                        new VzPoints.Style(Color.green, 1)));
                vb.swap();
            }

            double[][] evec = getGripAxes(rorigin);
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

    public BoltArmCommandInterpreter()
    {
        this(false);
    }

    public BoltArmCommandInterpreter(boolean debug_)
    {
        debug = debug_;
        if (debug) {
            dthread = new DebugThread();
            dthread.start();
        }

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
            cmds.add(cmd);
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
            bcmd.xyz[2] = Bolt.getCamera().getHeight(bcmd.xyz) + defaultPointHeight;
        } else {
            // Found ID
            int objID = Integer.valueOf(objIDstr);
            if (debug) {
                System.out.println("POINT @ "+objID);
            }
            bcmd.obj_id = objID;

            ObjectInfo info = getObject(objID);
            if (info == null) {
                System.out.println("NULL OBJECT");
                bcmd.xyz = LinAlg.resize(cmd.dest, 3);
            } else {
                if (debug) {
                    dthread.render(info.points);
                }
                ArrayList<double[]> points = k2wPointAlign(info.points);
                bcmd.xyz = getCentroidXYZ(flattenPoints(points));
                bcmd.xyz[2] = BoltUtil.getZAt(points, bcmd.xyz) + pointOffset;//getMax(points, 2) + pointOffset;
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
                System.out.println("NULL OBJECT");
                return null;    // There is no safe way to grab nothing
            } else {
                if (debug) {
                    dthread.render(info.points);
                }
                ArrayList<double[]> wPoints = k2wPointAlign(info.points);
                ArrayList<double[]> xyPoints = flattenPoints(wPoints);
                double[] uxy = getMeanXY(xyPoints);
                bcmd.xyz = LinAlg.resize(uxy, 3);
                //double zMax = getMax(wPoints, 2);
                //double zMid = getMid(wPoints, 2);
                double zMax = BoltUtil.getZAt(wPoints, uxy);
                double zMid = zMax/2;
                double zMin = getMin(wPoints, 2);
                bcmd.xyz[2] = zMax - Math.min(zMax - zMid, grabOffset);

                heldHeight = bcmd.xyz[2] - zMin; // Height above the ground we grabbed at

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
        //System.out.printf("theta: [%f] ",theta);
        theta = MathUtil.mod2pi(theta);
        //System.out.printf("mod2pi: [%f] ",theta);
        double max = Math.toRadians(150.0);
        double min = Math.toRadians(-150.0);
        if (theta > 0 && theta > max) {
            theta = MathUtil.mod2pi(theta - Math.PI);
            //System.out.printf(" big: [%f]\n", theta);
            return theta;
        } else if (theta < 0 && theta < min) {
            theta = MathUtil.mod2pi(theta + Math.PI);
            //System.out.printf(" small: [%f]\n", theta);
            return theta;
        }
        //System.out.printf("\n");
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

        // Find height of place we're putting down object.
        // This is our naive attempt to stack reasonably.
        // In practice, should check along the whole
        // profile of the object as held to see what collisions
        // will occur
        double zHeight = Bolt.getCamera().getHeight(bcmd.xyz);

        // Drop height cannot be less than the drop offset. This prevents us
        // from trying to move the arm below the ground plane
        bcmd.xyz[2] = Math.max(zHeight + heldHeight + dropOffset, dropOffset);

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

    /** Instuct the arm to reset to its default position */
    private bolt_arm_command_t processHomeCommand(robot_command_t cmd)
    {
        bolt_arm_command_t bcmd = new bolt_arm_command_t();
        bcmd.cmd_id = messageID++;
        bcmd.action = "HOME";
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

        // Deal with case that the ranges are nearly equal
        double equalityThresh = 0.01;
        double dx = maxX - minX;
        double dy = maxY - minY;

        if (Math.abs(dx-dy) < equalityThresh) {
            return new double[][] {{1.0, 0}, {0, 1.0}}; // Sideways grip bias
        } else if (dx > dy) {
            return new double[][] {{1.0, 0}, {0, 1.0}};
        } else {
            return new double[][] {{0, 1.0}, {1.0, 0}};
        }
    }

    /** Return the maximum value of the supplied points for specified idx*/
    private double getMax(ArrayList<double[]> points, int idx)
    {
        double max = Double.MIN_VALUE;
        for (double[] p: points) {
            max = Math.max(p[idx], max);
        }

        return max;
    }

    /** Return the minimum value of the supplied points for specified idx*/

    private double getMin(ArrayList<double[]> points, int idx)
    {
        double min = Double.MAX_VALUE;
        for (double[] p: points) {
            min = Math.min(p[idx], min);
        }

        return min;
    }

    /** Return the average value of the supplied points at idx */
    private double getAvg(ArrayList<double[]> points, int idx)
    {
        if (points.size() < 1)
            return 0;

        double avg = 0;
        for (double[] p: points) {
            avg += p[idx];
        }

        return avg/points.size();
    }

    /** Return the average value of the supplied points at idx */
    private double getMid(ArrayList<double[]> points, int idx)
    {
        double min = Double.MAX_VALUE;
        double max = Double.MIN_VALUE;

        for (double[] p: points) {
            min = Math.min(min, p[idx]);
            max = Math.max(max, p[idx]);
        }

        return (max - min)/2;
    }

    /** Return the ObjectInfo for a relevant ID */
    private ObjectInfo getObject(int id)
    {
        // XXX Threading could be fun
        for (Integer key: Segment.getSingleton().objects.keySet()) {
            System.out.println(key);
            ObjectInfo info = Segment.getSingleton().objects.get(key);
            if (info.repID == id)
                return info;
        }
        return null;
    }



    // ==========================

    static public void main(String[] args)
    {
        BoltArmCommandInterpreter baci = new BoltArmCommandInterpreter(true);
    }
}
