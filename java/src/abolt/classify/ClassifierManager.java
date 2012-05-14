package abolt.classify;

import java.util.ArrayList;
import java.util.HashMap;

import april.config.Config;

import abolt.classify.Features.FeatureCategory;
import abolt.objects.BoltObject;

public class ClassifierManager {
	
	private HashMap<FeatureCategory, IClassifier> classifiers;
	
	public ClassifierManager(Config config){
        String colorDataFile = "", shapeDataFile = "", sizeDataFile = "";
		// Load .dat files
        try {
            colorDataFile = config.requireString("training.color_data");
            shapeDataFile = config.requireString("training.shape_data");
            sizeDataFile = config.requireString("training.size_data");
        } catch (Exception ex) {
            System.err.println("ERR: Could not load all .dat files");
            ex.printStackTrace();
        }
        
		classifiers = new HashMap<FeatureCategory, IClassifier>();
        classifiers.put(FeatureCategory.COLOR, new KNN(1, 6, colorDataFile, 0.2));
        classifiers.put(FeatureCategory.SHAPE, new ShapeKNN(10, 15, shapeDataFile, 1));
        classifiers.put(FeatureCategory.SIZE, new KNN(5, 2, sizeDataFile, 1));
        
        reloadData();
	}
	
	public ConfidenceLabel classify(FeatureCategory cat, BoltObject obj){
		IClassifier classifier = classifiers.get(cat);
		ArrayList<Double> features = obj.getFeatures(cat);
		if(features == null){
			return null;
		}
		ConfidenceLabel label;
		synchronized(classifier){
			label = classifier.classify(features);
		}
		return label;
	}
	
	public void addDataPoint(FeatureCategory cat, ArrayList<Double> features, String label){
		IClassifier classifier = classifiers.get(cat);
		synchronized(classifier){
			classifier.add(features, label);
		}
	}
	
	public void clearData(){
		for(IClassifier classifier : classifiers.values()){
			synchronized(classifier){
				classifier.clearData();
			}
		}
	}
	
	public void reloadData(){
		for(IClassifier classifier : classifiers.values()){
			synchronized(classifier){
				classifier.clearData();
				classifier.loadData();
			}
		}
	}
	
	public void updateObject(BoltObject object){
		for(FeatureCategory cat : FeatureCategory.values()){
			ArrayList<Double> features = object.getFeatures(cat);
			if(features != null){
				IClassifier classifier = classifiers.get(cat);
				synchronized(classifier){
					object.getLabels().updateLabel(cat, classifier.classify(features));
				}
			}
		}
	}
}
