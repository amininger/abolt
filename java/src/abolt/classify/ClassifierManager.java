package abolt.classify;

import java.util.*;

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

    // Singleton handling
    private static ClassifierManager singleton = null;
    public static ClassifierManager getSingleton()
    {
        if (singleton == null) {
            singleton = new ClassifierManager();
        }
        return singleton;
    }

    // Undo/redo functionality
    private Stack<StackEntry> undoStack = new Stack<StackEntry>();
    private Stack<StackEntry> redoStack = new Stack<StackEntry>();
    private class StackEntry
    {
        public CPoint point;
        public FeatureCategory cat;
        public String action;

        public StackEntry(CPoint point_, FeatureCategory cat_, String action_)
        {
            point = point_;
            cat = cat_;
            action = action_;
        }
    }

    // The classfiers
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

	public Classifications classify(FeatureCategory cat, BoltObject obj){
		IClassifier classifier = classifiers.get(cat);
		ArrayList<Double> features = obj.getFeatures(cat);
		if(features == null){
			return null;
		}
        Classifications classifications;
		synchronized(classifier){
			classifications =classifier.classify(features);
		}
		return classifications;
	}

	public void addDataPoint(FeatureCategory cat, ArrayList<Double> features, String label){
		IClassifier classifier = classifiers.get(cat);
		synchronized(classifier){
            CPoint point = new CPoint(label, features);
            StackEntry entry = new StackEntry(point, cat, "ADD");
            undoStack.push(entry);
			classifier.add(point);
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

    public boolean hasUndo()
    {
        return undoStack.size() > 0;
    }

    public boolean hasRedo()
    {
        return redoStack.size() > 0;
    }

    /** Undo function. Undoes the last action taken by the user */
    public void undo()
    {
        if (undoStack.size() < 1)
            return;
        StackEntry entry = undoStack.pop();

        if (entry.action.equals("ADD")) {
            classifiers.get(entry.cat).removeLast();
            redoStack.push(entry);
        } else {
            System.err.println("ERR: Unhandled undo case - "+entry.action);
        }
    }

    /** Redo function. Takes the last undone action and redoes it */
    public void redo()
    {
        if (redoStack.size() < 1)
            return;
        StackEntry entry = redoStack.pop();

        if (entry.action.equals("ADD")) {
            classifiers.get(entry.cat).add(entry.point);
            undoStack.push(entry);
        } else {
            System.err.println("ERR: Unhandled redo case - "+entry.action);
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

        int i = 0;
        synchronized (objManager.objects) {
            od = new object_data_t[objManager.objects.size()];
            for (BoltObject bo: objManager.objects.values()) {
                od[i] = new object_data_t();
                od[i].utime = utime;
                od[i].id = bo.getID();
                od[i].pos = bo.getPose();
                od[i].bbox = bo.getBBox();

                categorized_data_t[] cat_dat = new categorized_data_t[classifiers.size()];
                int j = 0;
                for (FeatureCategory fc: classifiers.keySet()) {
                    cat_dat[j] = new categorized_data_t();
                    cat_dat[j].cat = new category_t();
                    cat_dat[j].cat.cat = Features.getLCMCategory(fc);
                    IClassifier classifier = classifiers.get(fc);
                    Classifications cs = classifier.classify(bo.getFeatures(fc));
                    cs.sortLabels();    // Just to be nice
                    cat_dat[j].len = cs.size();
                    cat_dat[j].confidence = new double[cat_dat[j].len];
                    cat_dat[j].label = new String[cat_dat[j].len];
                    int k = 0;
                    for (Classifications.Label label: cs.labels) {
                        cat_dat[j].confidence[k] = label.weight;
                        cat_dat[j].label[k] = label.label;

                        k++;
                    }

                    j++;
                }

                od[i].num_cat = cat_dat.length;
                od[i].cat_dat = cat_dat;

                i++;
            }
        }

        return od;
    }
}
