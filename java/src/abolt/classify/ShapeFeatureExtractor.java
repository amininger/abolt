package abolt.classify;

import java.awt.image.BufferedImage;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

import abolt.bolt.BoltObject;
import abolt.kinect.*;
/**
 * @author Aaron
 * Contains methods for extracting shape features from an object or point cloud
 */
public class ShapeFeatureExtractor
{
	public static ArrayList<Double> getFeatures(BoltObject obj) {
		return getFeatures(obj.getProjection());
	}
//
//	public static ArrayList<Double> getFeatures(ArrayList<double[]> points) {
//		BufferedImage img = ObjectInfo.getImage(points, null);
//		return getFeatures(img);
//	}

	public static ArrayList<Double> getFeatures(BufferedImage img){
		ArrayList<Double> features = PCA.getFeatures(img, 7);

        return features;    // XXX Rotations and stuff?
	}
}
