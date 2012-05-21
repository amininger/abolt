package abolt.classify;

import java.awt.Color;
import java.util.ArrayList;

import abolt.kinect.ObjectInfo;

/**
 * @author Aaron
 * @purpose Contains methods for extracting color features from an object or point cloud
 */
public class ColorFeatureExtractor{
	public static ArrayList<Double> getFeatures(ObjectInfo object) {
		return getFeatures(object.points);
	}

	public static ArrayList<Double> getFeatures(ArrayList<double[]> points) {
		ArrayList<Double> features = new ArrayList<Double>();
		FEUtil.addArray(features, avgRGB(points));
		FEUtil.addArray(features, avgHSV(points));
		return features;
	}
	
	/**
	 * Find the average red, green, and blue values for a group of pixels.
	 * pixels are assumed to have four coordinates, (x, y, z, rgb).
	 **
	 * @return [r,g,b] averages.
	 **/
	public static double[] avgRGB(ArrayList<double[]> points)
    {
		double[] avg = new double[3];
		for (double[] p : points) {
			Color bgr = new Color((int) p[3]);
			avg[2] += bgr.getRed();
			avg[1] += bgr.getGreen();
			avg[0] += bgr.getBlue();
		}
		FEUtil.divideEquals(avg, 255.0 * points.size());
		return avg;
	}

	/**
	 * Calculate the variance of red, green, blue values in a group of pixels.
	 * Pixels are assumed to have four coordinates: (x, y, z, rgb).
	 **
	 * @return [r,g,b,] variances
	 **/
	public static double[] varRGB(ArrayList<double[]> points)
    {
		double[] avg = avgRGB(points);
		double[] var = new double[3];
		for (double[] p : points) {
			Color bgr = new Color((int) p[3]);
			var[2] += (bgr.getRed() - avg[0]) * (bgr.getRed() - avg[0]);
			var[1] += (bgr.getGreen() - avg[1]) * (bgr.getGreen() - avg[1]);
			var[0] += (bgr.getBlue() - avg[2]) * (bgr.getBlue() - avg[2]);
		}
		FEUtil.divideEquals(var, points.size());
		return var;
	}

	/**
	 * Find the average hue, saturation, and values for a group of pixels.
	 * pixels are assumed to have four coordinates, (x, y, z, rgb).
	 **
	 * @return [h,s,v] averages.
	 **/
	public static double[] avgHSV(ArrayList<double[]> points)
    {
		double[] avg = new double[3];
		float[] hsv = new float[3];
		for (double[] p : points) {
			Color rgb = new Color((int) p[3]);
			Color.RGBtoHSB(rgb.getBlue(), rgb.getGreen(), rgb.getRed(), hsv);
			for (int i = 0; i < avg.length; i++) {
				avg[i] += (double) hsv[i];
			}
		}
		FEUtil.divideEquals(avg, points.size());
		return avg;
	}

	/**
	 * Calculate the variance of hue, saturation, and values in a group of
	 * pixels. Pixels are assumed to have four coordinates: (x, y, z, rgb).
	 **
	 * @return [h,s,v] variances
	 **/
	public static double[] varHSV(ArrayList<double[]> points)
    {
		double[] avg = avgHSV(points);
		double[] var = new double[3];
		float[] hsvV = new float[3];
		for (double[] p : points) {
			Color rgb = new Color((int) p[3]);
			Color.RGBtoHSB(rgb.getBlue(), rgb.getGreen(), rgb.getRed(), hsvV);
			for (int i = 0; i < avg.length; i++) {
				double hsv = (double) hsvV[i];
				var[i] += (hsv - avg[i]) * (hsv - avg[i]);
			}
		}
		FEUtil.divideEquals(var, points.size());
		return var;
	}
	
	public static Color getColorFromFeatures(ArrayList<Double> colorFeatures){
		return new Color((float)colorFeatures.get(0).doubleValue(),
				         (float)colorFeatures.get(1).doubleValue(),
				         (float)colorFeatures.get(2).doubleValue());
	}
}
