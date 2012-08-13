package abolt.kinect;

import java.awt.*;
import java.util.*;

import april.jmat.*;
import april.jmat.geom.*;
import april.util.UnionFindSimple;
import april.vis.*;
import april.config.*;

import abolt.arm.*;

public class Segment
{
    private static Segment singleton = null;
    public static Segment getSingleton()
    {
        if (singleton == null) {
            singleton = new Segment();
        }
        return singleton;
    }


    final static double COLOR_THRESH = .25;//30;
    final static double DISTANCE_THRESH = 0.01;
    final static double RANSAC_THRESH = .015;
    final static double RANSAC_PERCENT = .2;
    final static double MIN_OBJECT_SIZE = 100;
    final static int MAX_HISTORY = 100;

    static BoltArm ba;
    static Config config;
    ArrayList<Double> ARM_WIDTH;
    double MAX_HEIGHT;

    int width, height;

    // Originally in data aggregator
    public HashMap<Integer, ObjectInfo> objects;           //map of all objects found in current frame to their data
    public HashMap<Integer, Integer> map;                  //map of object ID to color
    public ArrayList<double[]> coloredPoints;
    public ArrayList<double[]> points;
    public UnionFindSimple ufs;
    public ObjectTracking tracker;
    public VisWorld.Buffer vb;

    static Random rand = new Random();
    static double[] t = new double[] { -0.0254, -0.00013, -0.01218 }; // Added .01 to t[2] here rather than below
    double[] floorPlane;
    boolean floorFound;
    // Set up some "Random" colors to draw the segments
    static int[] colors = new int[]{0xff3300CC, 0xff9900CC, 0xffCC0099, 0xffCC0033,
                                    0xff0033CC, 0xff470AFF, 0xff7547FF, 0xffCC3300,
                                    0xff0099CC, 0xffD1FF47, 0xffC2FF0A, 0xffCC9900,
                                    0xff00CC99, 0xff00CC33, 0xff33CC00, 0xff99CC00};

    public Segment()
    {
        tracker = new ObjectTracking();
        floorPlane = new double[4];
        floorFound = false;
        objects = new HashMap<Integer, ObjectInfo>();
        map = new HashMap<Integer, Integer>();
        coloredPoints = new ArrayList<double[]>();
        ba = BoltArm.getSingleton();
    }


    /** First segment the frame into objects and then get the features
     ** for each object. **/
    public void segmentFrame(ArrayList<double[]> currentPoints, int w, int h)
    {
//        if(ARM_WIDTH == null)
            ARM_WIDTH = ba.getArmWidths();

        width = w;
        height = h;
        points = currentPoints;
        coloredPoints.clear();
        removeFloorAndArmPoints();
        unionFind();
    }

    /** union find- for each pixel, compare with pixels around it and merge if
     ** they are close enough. **/
    public void unionFind()
    {
        ufs = new UnionFindSimple(points.size());
        //create unions of pixels that are close together spatially
        for(int y=0; y<height; y++){
            for(int x=0; x<width; x++){
                int loc1 = y*width + x;
                double[] p1 = points.get(loc1);
                // Look at neighboring pixels
                if(!almostZero(p1)){
                    int loc2 = y*width + x + 1;
                    int loc3 = (y+1)*width + x;

                    if (loc2>=0 && loc2<points.size() && (x+1)<width){
                        double[] p2 = points.get(loc2);
                        if(!almostZero(p2)
                           && (dist(p1, p2) < DISTANCE_THRESH
                               && colorDiff(p1[3], p2[3]) < COLOR_THRESH)){
                            ufs.connectNodes(loc1, loc2);
                        }
                    }

                    if (loc3>=0 && loc3<points.size() && (y+1)<height){
                        double[] p2 = points.get(loc3);
                        if(!almostZero(p2)
                           && (dist(p1, p2) < DISTANCE_THRESH
                               && colorDiff(p1[3], p2[3]) < COLOR_THRESH)){
                            ufs.connectNodes(loc1, loc3);
                        }
                    }
                }
            }
        }

        //collect data on all the objects segmented by the union find algorithm in the previous step
        ObjectInfo info;
        objects = new HashMap<Integer, ObjectInfo>();
        map = new HashMap<Integer, Integer>();

        // Make new objectInfos
        for(int i = 0; i < points.size(); i++){
            double[] point = points.get(i);
            if(!almostZero(point) && ufs.getSetSize(i) > MIN_OBJECT_SIZE){

                int repID = ufs.getRepresentative(i);
                Object repColor = map.get(repID);
                if(repColor != null){
                    info = (ObjectInfo)objects.get(repID);
                    info.update(point);
                }
                else{
                    int color = colors[i%colors.length];
                    map.put(repID, color);
                    info = new ObjectInfo(color, repID, point);
                    objects.put(repID, info);
                }
                Integer color = map.get(repID);
                coloredPoints.add(point);
            }
        }
        objects = tracker.newObjects(objects);
    }

    private boolean almostBlack(int color)
    {
        Color c = new Color(color);

        int rg = Math.abs(c.getRed()-c.getGreen());
        int rb = Math.abs(c.getRed()-c.getBlue());
        int gb = Math.abs(c.getGreen()-c.getBlue());

        if(rg+rb+gb < 18 && rg < 60) return true;
        return false;
    }


    /** "Remove" points that are too close to the floor by setting them to
     ** empty arrays.
     ** @return whether a plane was found and points were removed
     **/
    private boolean removeFloorAndArmPoints()
    {
        // Only calculate the floor plane once XXX - maybe do multiple times and average?
        if(floorFound == false){
            floorPlane = estimateFloor(2000);
            floorFound = true;
        }

        if (Arrays.equals(floorPlane, new double[4])) return false;

        // Figure out where each of the joints are
        ArrayList<double[]> armPoints = new ArrayList<double[]>();
        ArrayList<Joint> joints = tracker.arm.getJoints();

        double[][] xform = LinAlg.translate(0,0,tracker.arm.getBaseHeight());
        armPoints.add(new double[3]);
        armPoints.add(LinAlg.matrixToXyzrpy(xform));
        for (Joint j: joints) {
            LinAlg.timesEquals(xform, j.getRotation());
            LinAlg.timesEquals(xform, j.getTranslation());
            armPoints.add(LinAlg.matrixToXyzrpy(xform));
        }


        // Set up segments of the arm
        ArrayList<GLineSegment2D> armLines = new ArrayList<GLineSegment2D>();
        for(int i=1; i<armPoints.size(); i++){
            armLines.add(new GLineSegment2D(armPoints.get(i-1), armPoints.get(i)));
            double[] p1 = armPoints.get(i-1);
            double[] p2 = armPoints.get(i);

//            System.out.printf("%d: (%.4f, %.4f, %.4f) --> (%.4f, %.4f, %.4f)   %.2f\n",
//                              i, p1[0], p1[1], p1[2],p2[0], p2[1], p2[2], ARM_WIDTH.get(i-1));
        }

        // Draw cylinders
        /*      xform = LinAlg.translate(0,0,ba.baseHeight);
        for (int i=0; i<joints.size(); i++){ //Joint j: joints) {
            Joint j = joints.get(i);
            LinAlg.timesEquals(xform, j.getRotation());
            GLineSegment2D line = armLines.get(i);
            double[] p1 = new double[]{line.p1[0], line.p1[1], line.p1[2]};
            double[] p2 = new double[]{line.p2[0], line.p2[1], line.p2[2]};
            double length = LinAlg.distance(p1,p2);
            VzCylinder cyl = new VzCylinder(ARM_WIDTH.get(i), length, new VzMesh.Style(Color.red));
            vb.addBack(new VisChain(LinAlg.copy(xform),
                                    cyl));
            LinAlg.timesEquals(xform, j.getTranslation());
        }
        vb.swap();
        */

        /*for (int seg=0; seg<ARM_WIDTH.size(); seg++){
            GLineSegment2D line = armLines.get(seg);
            double[] p1 = new double[]{line.p1[0], line.p1[1], line.p1[2]};
            double[] p2 = new double[]{line.p2[0], line.p2[1], line.p2[2]};
            double length = LinAlg.distance(p1,p2);
            double[] diff = LinAlg.subtract(p2, p1);
            double[] center = new double[]{p1[0]+diff[0],p1[1]+diff[1],p1[2]+diff[2]};
            VzCylinder cyl = new VzCylinder(ARM_WIDTH.get(seg), length, new VzMesh.Style(Color.red));
            vb.addBack(new VisChain(LinAlg.translate(p1),
                                    //LinAlg.rotate(Math.)
                                    cyl));

        }
        vb.swap();*/


        //Draw lines corresponding to the segments of the arm.
//        vb.addBack(new VzLines(new VisVertexData(armPoints), 4, new VzLines.Style(Color.red, 1)));
//          vb.swap();


        // Remove points that are either on the floor plane, below the plane,
        // or along the arm's position
        for(int i=0; i<points.size(); i++){
            double[] point = points.get(i);
            double[] p = new double[]{point[0], point[1], point[2]};
            double distToPlane = pointToPlaneDist(p, floorPlane);
            if(distToPlane < RANSAC_THRESH ||
               distToPlane > ba.wristHeight ||
               belowPlane(p, floorPlane))// ||
                //inArmRange(armLines, p))// ||//almostBlack((int)point[3]))
                points.set(i, new double[4]);
            else if(inArmRange(armLines, KUtils.getWorldCoordinates(p)))
                points.set(i, new double[4]);
        }

        return true;
    }

    /** Determine whether a line is part of the arm, given a set of lines
     *  representing the positions of the joints.**/
    private boolean inArmRange(ArrayList<GLineSegment2D> armLines, double[] p)
    {
        assert(ARM_WIDTH.size() <= armLines.size());
        boolean inArm = false;
        for(int i=0; i<ARM_WIDTH.size(); i++){
            if(armLines.get(i).distanceTo(p) < ARM_WIDTH.get(i)){
                inArm = true;
                break;
            }
        }

//        if(inArm) System.out.print(".");
//        else System.out.printf("(%.2f,%.2f,%.3f)",p[0], p[1], p[2]);
        return inArm;
    }

    /** For determining whether a point is basically at the origin.**/
    private boolean almostZero(double[] p)
    {
        if(p[0] < .0001 && p[1] < .0001 && p[2] < .0001 && p[3] < .0001)
            return true;
        return false;
    }


    /** Check if a given point is on the other side of the ground plane as
     ** the camera is (this might mean we want to delete them).**/
    private boolean belowPlane(double[] p, double[] coef)
    {
        double eval = coef[0]*p[0] + coef[1]*p[1] + coef[2]*p[2] + coef[3];
        if(eval < 0 && coef[3] > 0) return true;
        else if(eval > 0 && coef[3] < 0) return true;
        return false;
    }


    /** Given a point and the coefficients for a plane, find the distance
     ** between them.
     ** @param point is the point we want a distance to
     ** @param coef is the pqrs coeficients of a plane (px+qy+rz+s=0)
     ** @return distance
     **/
    private double pointToPlaneDist(double[] point, double[] coef)
    {
        assert(point.length == 3 && coef.length == 4);

        double ax = coef[0]*point[0];
        double by = coef[1]*point[1];
        double cz = coef[2]*point[2];
        double a2 = coef[0]*coef[0];
        double b2 = coef[1]*coef[1];
        double c2 = coef[2]*coef[2];

        return Math.abs(ax+ by + cz + coef[3])/ Math.sqrt(a2 + b2 + c2);
    }

    /** Get the difference in the z-direction of two pixels. **/
    private double depthDiff(double[] p1, double[] p2)
    {
        return Math.abs(p1[2] - p2[2]);
    }

    /** Get the difference in the z-direction of two pixels. **/
    private double dist(double[] p1, double[] p2)
    {
        double dx = p1[0]-p2[0];
        double dy = p1[1]-p2[1];
        double dz = p1[2]-p2[2];
        return Math.sqrt(dx*dx + dy*dy + dz*dz);
    }

    /** Find the Euclidean distance between two colors. **/
    private double colorDiff(double color1, double color2)
    {
        Color c1 = new Color((int)color1);
        Color c2 = new Color((int)color2);

        //int rDiff = c1.getRed() - c2.getRed();
        //int gDiff = c1.getGreen() - c2.getGreen();
        //int bDiff = c1.getBlue() - c2.getBlue();
        //double diff = Math.sqrt(rDiff*rDiff + bDiff*bDiff + gDiff*gDiff);

        float[] hsv1 = Color.RGBtoHSB(c1.getRed(), c1.getGreen(), c1.getBlue(), null);
        float[] hsv2 = Color.RGBtoHSB(c2.getRed(), c2.getGreen(), c2.getBlue(), null);
        double diff = Math.abs((double)hsv1[0] - (double)hsv2[0]);

        return diff;
    }

    /** Estimate the floor plane using RANSAC algorithm (assumes that the major
        plane in the image is the "floor").
        @param iterations is the number of iterations RANSAC is run for
        @return The characterizing coefficients for the floor plane
    **/
    public double[] estimateFloor(int iterations)
    {
        if(points.size() == 0)
            return null;

        int numPoints = points.size();
        double bestPlane[] = new double[4];  // Parameters of plane equation
        int bestFit = 0;                     // Most points that fit a guess plane
        int numSamples = (int)Math.floor(numPoints*RANSAC_PERCENT);

        // Perform specified number of iterations
        for(int i=0; i<iterations; i++){
            int numFit = 0;
            // Choose three random points
            double[] r1 = points.get(rand.nextInt(numPoints));
            double[] r2 = points.get(rand.nextInt(numPoints));
            double[] r3 = points.get(rand.nextInt(numPoints));

            double[] p1 = new double[]{r1[0], r1[1], r1[2]};
            double[] p2 = new double[]{r2[0], r2[1], r2[2]};
            double[] p3 = new double[]{r3[0], r3[1], r3[2]};

            // Derive plane through all three points
            double[] p2p1 = LinAlg.subtract(p2, p1);
            double[] p3p1 = LinAlg.subtract(p3, p1);
            double[] pqr = LinAlg.crossProduct(p2p1, p3p1);
            double s = -(pqr[0]*r1[0] + pqr[1]*r1[1] + pqr[2]*r1[2]);
            double[] plane = new double[]{pqr[0], pqr[1], pqr[2], s};

            // Check whether a sample of points is within a threshold of the plane.
            for(int j = 0; j < numSamples; j++){
                double[] p = points.get(rand.nextInt(numPoints));
                if(Math.abs(p[0]) < t[2])
                    continue;
                if (pointToPlaneDist(new double[]{p[0], p[1], p[2]}, plane) < .005)
                    numFit ++;
            }

            // Compare new plane against last best, saving it if better
            if(numFit > bestFit && numFit > (numSamples/6)){
                bestFit = numFit;
                bestPlane = LinAlg.copy(plane);
            }
        }

        double percent = bestFit / (double) numSamples;

        if(bestFit == 0)
            return new double[4];
        return bestPlane;
    }
}
