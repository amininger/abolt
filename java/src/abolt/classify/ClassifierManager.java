package abolt.classify;

import java.util.ArrayList;
import java.util.HashMap;

import april.config.*;
import april.util.*;

import abolt.classify.Features.FeatureCategory;
import abolt.objects.*;
import abolt.lcmtypes.*;

/**
 * @author aaron
 * Creates the classifiers used in the system and acts as a point of contact to them
 */
public class ClassifierManager {
    private static ClassifierManager singleton = null;
    public static ClassifierManager getSingleton()
    {
        if (singleton == null) {
            singleton = new ClassifierManager();
        }
        return singleton;
    }

	private HashMap<FeatureCategory, IClassifier> classifiers;

    public ClassifierManager()
    {
    }

	public ClassifierManager(Config config)
    {
        addClassifiers(config);
	}

    public void addClassifiers(Config config)
    {
        String colorDataFile = null, shapeDataFile = null, sizeDataFile = null;
        colorDataFile = config.getString("training.color_data");
        shapeDataFile = config.getString("training.shape_data");
        sizeDataFile = config.getString("training.size_data");

		classifiers = new HashMap<FeatureCategory, IClassifier>();
        //classifiers.put(FeatureCategory.COLOR, new KNN(1, 6, colorDataFile, 0.2));
        //classifiers.put(FeatureCategory.SHAPE, new ShapeKNN(10, 15, shapeDataFile, 1));
        //classifiers.put(FeatureCategory.SIZE, new KNN(5, 2, sizeDataFile, 1));

        // New classification code
        // For now, manually specified parameters on weight

        GKNN colorKNN = new GKNN(10, 0.1);
        colorKNN.setDataFile(colorDataFile);
        classifiers.put(FeatureCategory.COLOR, colorKNN);

        GKNN shapeKNN = new GKNN(10, .3);      // XXX Untested parameter
        shapeKNN.setDataFile(shapeDataFile);
        classifiers.put(FeatureCategory.SHAPE, shapeKNN);

        GKNN sizeKNN  = new GKNN(5, .00003);      // XXX Needs revisiting, both in terms of
        sizeKNN.setDataFile(sizeDataFile);      // XXX parameter and classification

        classifiers.put(FeatureCategory.SIZE, sizeKNN);

        reloadData();
    }
    
    public void addClassifier(FeatureCategory cat, IClassifier classifier){
    	classifiers.put(cat, classifier);
    }

	public Classifications classify(FeatureCategory cat, BoltObject obj){
		IClassifier classifier = classifiers.get(cat);
		ArrayList<Double> features = obj.getFeatures(cat);
		if(features == null || classifier == null){
			return null;
		}
        Classifications classifications;
		synchronized(classifier){
			classifications = classifier.classify(features);
		}
		return classifications;
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

    /** Build up the object_data_t describing the observed objects
     *  in the world. Runs classifiers on the objects and builds
     *  the appropriate lcmtypes to return.
     */
    public object_data_t[] getObjectData()
    {
        BoltObjectManager objManager = BoltObjectManager.getSingleton();
        object_data_t[] od;
        long utime = TimeUtil.utime();
    	ArrayList<object_data_t> objectDatas = new ArrayList<object_data_t>();

        int i = 0;
        synchronized (objManager.objects) {
            od = new object_data_t[objManager.objects.size()];
            for (BoltObject bo: objManager.objects.values()) {
            	if(!bo.isVisible()){
            		continue;
            	}
            	object_data_t objData = new object_data_t();
            	objData.utime = utime;
            	objData.id = bo.getID();
            	objData.pos = bo.getPose();
            	objData.bbox = bo.getBBox();
            	ArrayList<categorized_data_t> cat_dats = new ArrayList<categorized_data_t>();
                int j = 0;
                for (FeatureCategory fc: classifiers.keySet()) {
                    ArrayList<Double> features = bo.getFeatures(fc);
                    IClassifier classifier = classifiers.get(fc);
                    if(features == null || classifier == null){
                    	continue;
                    }
                	categorized_data_t cat_dat = new categorized_data_t();
                    cat_dat = new categorized_data_t();
                    cat_dat.cat = new category_t();
                    cat_dat.cat.cat = Features.getLCMCategory(fc);
                    // AM: I don't like this, but I don't want to deal with features from visual properties at the moment
                    if(fc == FeatureCategory.WEIGHT){
                        cat_dat.num_features = features.size();
                        cat_dat.features = new double[features.size()];
                        for(int k = 0; k < features.size(); k++){
                        	cat_dat.features[k] = features.get(k);
                        }
                    } else {
                    	cat_dat.num_features = 0;
                    	cat_dat.features = new double[0];
                    }
                    Classifications cs = classifier.classify(features);
                    cs.sortLabels();    // Just to be nice
                    cat_dat.len = cs.size();
                    cat_dat.confidence = new double[cat_dat.len];
                    cat_dat.label = new String[cat_dat.len];
                    int k = 0;
                    for (Classifications.Label label: cs.labels) {
                        cat_dat.confidence[k] = label.weight;
                        cat_dat.label[k] = label.label;

                        k++;
                    }
                    
                    cat_dats.add(cat_dat);
                }

                objData.num_cat = cat_dats.size();
                objData.cat_dat = cat_dats.toArray(new categorized_data_t[cat_dats.size()]);
                objectDatas.add(objData);
            }
        }

        return objectDatas.toArray(new object_data_t[objectDatas.size()]);
    }
}
