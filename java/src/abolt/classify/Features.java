package abolt.classify;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

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
		COLOR, SHAPE, SIZE, WEIGHT
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

	// Mapping from the FeatureCategory to a FeatureExtractor
	public static ArrayList<Double> getFeatures(FeatureCategory cat,
			ObjectInfo object) {
		return getFeatures(cat, object.points);
	}

	public static ArrayList<Double> getFeatures(FeatureCategory cat,
			ArrayList<double[]> points) {
		switch (cat) {
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
