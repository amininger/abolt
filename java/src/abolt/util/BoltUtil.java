package abolt.util;

import java.util.*;

import april.jmat.*;

/** A general package of utility functions for
 *  use in various Bolt classes.
 */
public class BoltUtil
{
    // === Utility classes ===
    /** Key for plane isolation. Bins are centered
     *  around values and extend for +/- (r/2) to
     *  either direction
     */
    static private class BinKey
    {
        double r;
        int bin;

        public BinKey(double r_, double z_)
        {
            r = r_;

            double b = Math.floor(z_/r_);
            double delta = z_ - b;
            bin = (int)b;
            if (delta > (r_/2)) {
                bin += 1;
            }
        }

        public int hashCode()
        {
            return Integer.valueOf(bin).hashCode();
        }

        public boolean equals(Object o)
        {
            if (o == null)
                return false;
            if (!(o instanceof BinKey))
                return false;
            BinKey bk = (BinKey)o;
            return (bin == bk.bin);
        }
    }

    /** Transform the supplied points such that the
     *  area of the box bounding them is minimized
     *  in XY and such that the X coordinate of the
     *  box is maximized
     */
    static public ArrayList<double[]> getCanonical(ArrayList<double[]> points)
    {
        double theta = getBBoxTheta(points);
        double[] cxyz = getCentroidXY(points);
        double[][] cXform = LinAlg.translate(cxyz);
        ArrayList<double[]> centered = LinAlg.transform(cXform, points);
        ArrayList<double[]> rotated = LinAlg.transform(LinAlg.rotateZ(Math.toRadians(theta)),
                                                       centered);
        double minX = Double.MAX_VALUE;
        double minY = Double.MAX_VALUE;
        double maxX = Double.MIN_VALUE;
        double maxY = Double.MIN_VALUE;
        for (double[] p: rotated) {
            minX = Math.min(minX, p[0]);
            maxX = Math.max(maxX, p[0]);
            minY = Math.min(minY, p[1]);
            maxY = Math.max(maxY, p[1]);
        }

        // Flip the points such that they are in the "canonical"
        // orientation for this shape. This is accomplished by
        // minimizing the distance from the bottom left corner
        // to the centroid
        double lx = cxyz[0] - minX;
        double rx = maxX - cxyz[1];
        double by = cxyz[1] - minY;
        double ty = maxY - cxyz[1];

        double sx = rx > lx ? -1 : 1;
        double sy = ty > by ? -1 : 1;

        rotated = LinAlg.transform(LinAlg.scale(sx, sy, 1), rotated);

        return rotated;
    }

    /** Return the rotation around the XY centroid necessary to
     *  minimize the area of the axis-aligned bounding box. Since
     *  several such rotations should exist, pick the orientation
     *  with the greatest spread of points along the X-axis.
     */
    static public double getBBoxTheta(ArrayList<double[]> points)
    {
        double[] cxyz = getCentroidXY(points);
        double[][] cXform = LinAlg.translate(cxyz);
        ArrayList<double[]> centered = LinAlg.transform(LinAlg.inverse(cXform), points);

        // Rotate the points incrementally and calculate the area of the bbox.
        // There should be several orientations in which the box is minimized,
        // so choose the one resulting in the longested distribution along the
        // x-axis
        double minArea = Double.MAX_VALUE;
        double xlen = 0;
        double theta = 0;
        for (double r = 0; r < 360.0; r += 0.25)
        {
            ArrayList<double[]> rotated = LinAlg.transform(LinAlg.rotateZ(Math.toRadians(r)),
                                                           centered);

            // Find the area of the bounding box
            double minX = Double.MAX_VALUE;
            double maxX = Double.MIN_VALUE;
            double minY = Double.MAX_VALUE;
            double maxY = Double.MIN_VALUE;
            for (double[] p: rotated) {
                minX = Math.min(minX, p[0]);
                maxX = Math.max(maxX, p[0]);
                minY = Math.min(minY, p[1]);
                maxY = Math.max(maxY, p[1]);
            }

            double area = (maxX - minX) * (maxY - minY);
            if (area < minArea) {
                minArea = area;
                theta = Math.toRadians(r);
                xlen = (maxX - minX);
            }
        }

        return theta;
    }

    /** Move the supplied points to the origin and rotate
     *  by the requested angle
     */
    static public ArrayList<double[]> rotateAtOrigin(ArrayList<double[]> points, double theta)
    {
        double[] cxyz = getCentroidXY(points);
        double[][] cXform = LinAlg.translate(cxyz);
        ArrayList<double[]> centered = LinAlg.transform(LinAlg.inverse(cXform), points);

        return LinAlg.transform(LinAlg.rotateZ(theta), centered);
    }

    /** Rotate the supplied points in place around their
     *  XY centroid
     */
    static public ArrayList<double[]> rotateInPlace(ArrayList<double[]> points, double theta)
    {
        double[] cxyz = getCentroidXY(points);
        double[][] cXform = LinAlg.translate(cxyz);
        ArrayList<double[]> centered = LinAlg.transform(LinAlg.inverse(cXform), points);
        ArrayList<double[]> rotated = LinAlg.transform(LinAlg.rotateZ(theta), centered);

        return LinAlg.transform(cXform, rotated);
    }

    /** Return the centroid of the supplied points */
    static public double[] getCentroid(ArrayList<double[]> points)
    {
        if (points == null || points.size() < 1)
            return null;

        double[] mean = LinAlg.copy(points.get(0));
        for (int i = 1; i < points.size(); i++) {
            LinAlg.add(mean, points.get(i), mean);
        }

        LinAlg.scale(mean, 1.0/points.size(), mean);

        return mean;
    }

    /** Return the XY centroid of the supplied points based
     *  on the upper face of the shape
     */
    static public double[] getCentroidXY(ArrayList<double[]> points)
    {
        if (points == null || points.size() < 1)
            return null;

        return getCentroid(isolateTopFace(points));
    }

    static public double getZAt(ArrayList<double[]> points, double x, double y)
    {
        return getZAt(points, new double[] {x,y});
    }

    static public double getZAt(ArrayList<double[]> points, double[] xy)
    {
        double[] nearest = null;
        double minDist = Double.MAX_VALUE;
        double MAX_DIST = 0.02; // Point should be within 2 cm of sample request

        for (double[] p: points) {
            double dist = Math.sqrt(LinAlg.sq(xy[0]-p[0]) + LinAlg.sq(xy[1]-p[1]));
            if (dist < MAX_DIST && dist < minDist) {
                minDist = dist;
                nearest = p;
            }
        }

        if (nearest != null)
            return nearest[2];

        return 0;
    }

    static public ArrayList<double[]> isolateTopFace(ArrayList<double[]> points)
    {
        return isolateTopFace(points, 0.005);
    }

    /** Isolates the upper face of our observed shapes.
     *  Discretizes points in the Z dimension based on
     *  the r parameter and then returns all points within
     *  that discretization
     */
    static public ArrayList<double[]> isolateTopFace(ArrayList<double[]> points, double r)
    {
        assert (points.size() != 0);

        /*
        HashMap<BinKey, ArrayList<double[]> > bins = new HashMap<BinKey, ArrayList<double[]> >();
        for (double[] p: points) {
            BinKey key = new BinKey(r, p[2]);
            if (!bins.containsKey(key)) {
                bins.put(key, new ArrayList<double[]>());
            }
            bins.get(key).add(p);
        }

        BinKey maxKey = null;
        int max = 0;
        for (BinKey key: bins.keySet()) {
            int size = bins.get(key).size();
            if (size > max) {
                max = size;
                maxKey = key;
            }
        }

        assert (maxKey != null);

        return bins.get(maxKey);*/

        // Find a search range
        double min = Double.MAX_VALUE;
        double max = Double.MIN_VALUE;
        for (double[] p: points) {
            min = Math.min(p[2], min);
            max = Math.max(p[2], max);
        }

        double bestZ = max;
        int bestCnt = 0;

        for (double z = min; z <= max; z+= 0.001) {
            int cnt = 0;
            for (double[] p: points) {
                if (Math.abs(z-p[2]) < r)
                    cnt++;
            }
            if (cnt > bestCnt) {
                bestCnt = cnt;
                bestZ = z;
            }
        }

        ArrayList<double[]> top = new ArrayList<double[]>();
        for (double[] p: points) {
            if (Math.abs(bestZ-p[2]) < r) {
                top.add(p);
            }
        }

        return top;
    }

}
