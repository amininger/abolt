package abolt.classify;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

import abolt.kinect.ObjectInfo;
import abolt.lcmtypes.category_t;

public class Features {
	public enum FeatureCategory
    {
		COLOR, SHAPE, SIZE
	}
	
	// Mapping from a FeatureCategory to the category_t enum
	private static HashMap<FeatureCategory, Integer> featureToLCMCat;
	static{
		featureToLCMCat = new HashMap<FeatureCategory, Integer>();
		featureToLCMCat.put(FeatureCategory.COLOR, category_t.CAT_COLOR);
		featureToLCMCat.put(FeatureCategory.SHAPE, category_t.CAT_COLOR);
		featureToLCMCat.put(FeatureCategory.SIZE, category_t.CAT_COLOR);
	}
	public static Integer getLCMCategory(FeatureCategory cat){
		return featureToLCMCat.get(cat);
	}
	
	// Mapping from the LCM category_t.cat enum to a FeatureCategory
	private static HashMap<Integer, FeatureCategory> lcmToFeatureCat;
	static{
		lcmToFeatureCat = new HashMap<Integer, FeatureCategory>();
		lcmToFeatureCat.put(category_t.CAT_COLOR, FeatureCategory.COLOR);
		lcmToFeatureCat.put(category_t.CAT_COLOR, FeatureCategory.SHAPE);
		lcmToFeatureCat.put(category_t.CAT_COLOR, FeatureCategory.SIZE);
	}
	public static FeatureCategory getFeatureCategory(Integer lcmCat){
		return lcmToFeatureCat.get(lcmCat);
	}
	
	// Mapping from the FeatureCategory to a FeatureExtractor
	public static ArrayList<Double> getFeatures(FeatureCategory cat, ObjectInfo object){
		return getFeatures(cat, object.points);
	}
	
	public static ArrayList<Double> getFeatures(FeatureCategory cat, ArrayList<double[]> points){
		switch(cat){
		case COLOR:
			return ColorFeatureExtractor.getFeatures(points);
		case SIZE:
			return SizeFeatureExtractor.getFeatures(points);
		case SHAPE:
			return ShapeFeatureExtractor.getFeatures(points);
		}
		return null;
	}
}
