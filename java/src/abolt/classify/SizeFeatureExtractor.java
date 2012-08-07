package abolt.classify;

import java.util.*;

import april.jmat.*;

import abolt.bolt.*;
import abolt.kinect.*;
import abolt.util.*;

/**
 * @author Aaron
 * Contains methods for extracting size features from an object or point cloud
 */
public class SizeFeatureExtractor{
	public static ArrayList<Double> getFeatures(ObjectInfo object)
    {
		return getFeatures(object.points);
	}

    // XXX Original size features
	/*public static ArrayList<Double> getFeatures(ArrayList<double[]> points)
    {
		ArrayList<Double> features = new ArrayList<Double>();
		if(points.size() == 0){
			features.add(0.0);
			features.add(0.0);
			return features;
		}

		// Feature: Length of bbox diagonal
		double[] bbox = boundingBox(points);
		features.add(Math.sqrt(LinAlg.normF(new double[]{bbox[3] - bbox[0], bbox[4] - bbox[1], bbox[5] - bbox[2]})));

		// Feature: average distance from the mean
		double[] mean = new double[4];
		for(double[] pt : points){
			mean = LinAlg.add(mean, pt);
		}
		mean = LinAlg.scale(mean, 1.0/points.size());

		double distSum = 0;
		for(double[] pt : points){
			double[] diff = LinAlg.subtract(pt, mean);
			diff[3] = 0;
			distSum += Math.sqrt(LinAlg.normF(diff));
		}
		distSum /= points.size();

		features.add(distSum);

		return features;
	}*/

    // Slightly improved (???) size features
    public static ArrayList<Double> getFeatures(ArrayList<double[]> points)
    {
        // Now extract the appropriate values
        ArrayList<Double> features = new ArrayList<Double>();
		if(points.size() == 0){
			//features.add(0.0);
			features.add(0.0);
			return features;
		}

        // First, put the points into a canonical orientation
        // XXX Where does this conversion happen?
        ArrayList<double[]> worldPoints = KUtils.k2wConvert(points);
        ArrayList<double[]> canonical = BoltUtil.getCanonical(worldPoints);

		// Feature: Length of bbox diagonal
		//double[] bbox = boundingBox(canonical);
		//features.add(Math.sqrt(LinAlg.normF(new double[]{bbox[3] - bbox[0], bbox[4] - bbox[1], bbox[5] - bbox[2]})));

        // Feature: average distance from the mean
        double[] mean = BoltUtil.getCentroid(canonical);

		double distSum = 0;
		for(double[] pt : canonical){
			distSum += LinAlg.distance(pt, mean);
		}
		distSum /= points.size();

		features.add(distSum);

		return features;

    }

	/**
	 * Find the bounding box for a group of pixels by finding the extreme values
	 * in all directions of the points. This may not be the best way/may be
	 * implemented elsewhere.
	 **
	 * @return [xmin, ymin, zmin, xmax, ymax, zmax]
	 */
	public static double[] boundingBox(ArrayList<double[]> points) {
		double[] max = new double[] { -1000, -1000, -1000 };
		double[] min = new double[] { 1000, 1000, 1000 };
		for (double[] p : points) {
			for (int i = 0; i < 3; i++) {
				if (p[i] < min[i])
					min[i] = p[i];
				if (p[i] > max[i])
					max[i] = p[i];
			}
		}
		return new double[] { min[0], min[1], min[2], max[0], max[1], max[2] };
	}

	public static double[] boundingBoxWorld(ArrayList<double[]> points) {
		double[] max = new double[] { -1000, -1000, -1000 };
		double[] min = new double[] { 1000, 1000, 1000 };
		for (double[] p : points) {
            p = Bolt.getCamera().getWorldCoords(p);
			for (int i = 0; i < 3; i++) {
				if (p[i] < min[i])
					min[i] = p[i];
				if (p[i] > max[i])
					max[i] = p[i];
			}
		}
		return new double[] { min[0], min[1], min[2], max[0], max[1], max[2] };
	}


	/**
	 * Get the length, width, and height of the object by first getting the
	 * bounding box of an object. Length is in the x direction, width is in the
	 * y direction, and height is in z.
	 **
	 * @return [length, width, height] from bounds.
	 **/
	public static double[] lwh(ArrayList<double[]> points) {
		return lwh(boundingBox(points));
	}

	/**
	 * Get the length, width, and height of the object (as perceived). Length is
	 * in the x direction, width is in the y direction, and height is in z.
	 **
	 * @return [length, width, height] from bounds.
	 **/
	public static double[] lwh(double[] bounds)
    {
		assert (bounds.length == 6);
		double[] lwh = new double[3];
		lwh[0] = Math.abs(bounds[3] - bounds[0]);
		lwh[1] = Math.abs(bounds[4] - bounds[1]);
		lwh[2] = Math.abs(bounds[5] - bounds[2]);
		return lwh;
	}
}
