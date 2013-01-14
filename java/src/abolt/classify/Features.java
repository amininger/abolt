package abolt.classify;

import java.awt.Color;
import java.awt.Rectangle;
import java.awt.image.BufferedImage;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

import abolt.bolt.BoltObject;
import abolt.bolt.ObjectEffector;
import abolt.kinect.IBoltCamera;
import abolt.kinect.ObjectInfo;
import abolt.lcmtypes.category_t;

/**
 * @author aaron
 * @purpose Contains mappings between a FeatureCategory and the LCM category_t
 *          enum and a general interface for extracting features from a point
 *          cloud
 */
public class Features {
	public enum FeatureCategory {
		COLOR, SHAPE, SIZE, WEIGHT, SQUISHINESS
	}
	
	public static void addFeaturePair(FeatureCategory fcat, Integer lcmcat){
		featureToLCMCat.put(fcat, lcmcat);
		lcmToFeatureCat.put(lcmcat, fcat);
	}

	// Mapping from a FeatureCategory to the category_t enum
	private static HashMap<FeatureCategory, Integer> featureToLCMCat;
	static {
		featureToLCMCat = new HashMap<FeatureCategory, Integer>();
		featureToLCMCat.put(FeatureCategory.COLOR, category_t.CAT_COLOR);
		featureToLCMCat.put(FeatureCategory.SHAPE, category_t.CAT_SHAPE);
		featureToLCMCat.put(FeatureCategory.SIZE, category_t.CAT_SIZE);
	}

	public static Integer getLCMCategory(FeatureCategory cat) {
		return featureToLCMCat.get(cat);
	}

	// Mapping from the LCM category_t.cat enum to a FeatureCategory
	private static HashMap<Integer, FeatureCategory> lcmToFeatureCat;
	static {
		lcmToFeatureCat = new HashMap<Integer, FeatureCategory>();
		lcmToFeatureCat.put(category_t.CAT_COLOR, FeatureCategory.COLOR);
		lcmToFeatureCat.put(category_t.CAT_SHAPE, FeatureCategory.SHAPE);
		lcmToFeatureCat.put(category_t.CAT_SIZE, FeatureCategory.SIZE);
	}

	public static FeatureCategory getFeatureCategory(Integer lcmCat) {
		return lcmToFeatureCat.get(lcmCat);
	}
	
	private static ArrayList<ObjectEffector> effectors = new ArrayList<ObjectEffector>();
	public static void addEffector(ObjectEffector effector){
		effectors.add(effector);
	}
	
	public static void determineFeatures(BoltObject obj){
		if(obj.getPoints() == null){
			return;
		}
		obj.addFeature(FeatureCategory.COLOR, ColorFeatureExtractor.getFeatures(obj));
		obj.addFeature(FeatureCategory.SHAPE, ShapeFeatureExtractor.getFeatures(obj));
		obj.addFeature(FeatureCategory.SIZE, SizeFeatureExtractor.getFeatures(obj));
		for(ObjectEffector effector : effectors){
			effector.effect(obj);
		}
	}
	
    public static BufferedImage getImage(IBoltCamera camera, ArrayList<double[]> points){
    	BufferedImage image;
		int minX = Integer.MAX_VALUE, maxX = Integer.MIN_VALUE;
		int minY = Integer.MAX_VALUE, maxY = Integer.MIN_VALUE;
		for(double[] pt : points){
			int[] pixel = camera.getPixel(pt);
			if(pixel == null){
				continue;
			}
			minX = (pixel[0] < minX ? pixel[0] : minX);
			maxX = (pixel[0] > maxX ? pixel[0] : maxX);
			minY = (pixel[1] < minY ? pixel[1] : minY);
			maxY = (pixel[1] > maxY ? pixel[1] : maxY);
		}
		int margin = 5;
		Rectangle projBBox = new Rectangle();
		if(projBBox != null){
			projBBox.setBounds(minX - margin, minY - margin,
                               maxX - minX + 1 + margin*2,
                               maxY - minY + 1 + margin*2);
		}
		image = new BufferedImage((maxX - minX + 1) + 2*margin,
                                  (maxY - minY + 1) + 2*margin,
                                  BufferedImage.TYPE_3BYTE_BGR);
		for(int i = 0; i < points.size(); i++){
			int[] pixel = camera.getPixel(points.get(i));
			if(pixel == null){
				continue;
			}
			try{
				Color c =  new Color((int)points.get(i)[3]);
				Color rc = new Color(c.getBlue(), c.getGreen(), c.getRed());
    			image.setRGB(pixel[0]+margin-minX,
                             pixel[1]+margin-minY,
                             rc.getRGB());
			} catch (Exception e){
                e.printStackTrace();
			}
		}
    	return image;
    }

//	public static ArrayList<Double> getFeatures(FeatureCategory cat,
//			ArrayList<double[]> points) {
//		switch (cat) {
//		case COLOR:
//			return ColorFeatureExtractor.getFeatures(points);
//		case SIZE:
//			return SizeFeatureExtractor.getFeatures(points);
//		case SHAPE:
//			return ShapeFeatureExtractor.getFeatures(points);
//		default:
//			return null;
//		}
//	}
}
