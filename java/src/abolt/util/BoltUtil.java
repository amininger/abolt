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
        // Translate points to the origin
        ArrayList<double[]> topFace = isolateTopFace(points, 0.25);
        double[] cxyz = getCentroid(topFace);
        double[][] cXform = LinAlg.translate(-cxyz[0], -cxyz[1], 0);
        ArrayList<double[]> centered = LinAlg.transform(cXform, topFace);
        ArrayList<double[]> rotated;

        // Rotate the points incrementally and calculate the area of the bbox.
        // Chooses the minimal area that puts the shape in some canonical
        // orientation. In this case, we choose the orientation that shifts
        // our points closest to the edges that are down and to the left
        double minArea = Double.MAX_VALUE;
        double xlen = 0;
        double theta = 0;
        for (double r = 0; r < 360.0; r += 0.5)
        {
            rotated = LinAlg.transform(LinAlg.rotateZ(Math.toRadians(r)),
                                       centered);

            // Find the area of the bounding box
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

            // If the area is approximately minimal (within our threshold)
            // then look at the centroid position with relation to the
            // bounding box corner positions and choose the orientation
            // such that the centroid is closest to the bottom left corner
            // of the box.
            double area = (maxX - minX) * (maxY - minY);
            if (area < minArea || BoltMath.equals(area, minArea, 0.0000001)) {
                if (xlen < (maxX - minX)) {
                    minArea = area;
                    xlen = (maxX - minX);
                    theta = r;
                }
            }
        }

        centered = LinAlg.transform(cXform, points);
        rotated = LinAlg.transform(LinAlg.rotateZ(Math.toRadians(theta)),
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

    /** Isolates the upper face of our observed shapes.
     *  Discretizes points in the Z dimension based on
     *  the r parameter and then returns all points within
     *  that discretization
     */
    static public ArrayList<double[]> isolateTopFace(ArrayList<double[]> points, double r)
    {
        assert (points.size() != 0);

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

        return bins.get(maxKey);
    }

}
