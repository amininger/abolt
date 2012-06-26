package abolt.classify;

import java.util.*;
import java.io.*;

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
    Object stateLock = new Object();
    private LinkedList<StackEntry> undoStack = new LinkedList<StackEntry>();
    private LinkedList<StackEntry> redoStack = new LinkedList<StackEntry>();
    private class StackEntry
    {
        public CPoint point;
        public FeatureCategory cat;
        public String action;

        // Only used when reading in
        public StackEntry()
        {

        }

        public StackEntry(CPoint point_, FeatureCategory cat_, String action_)
        {
            point = point_;
            cat = cat_;
            action = action_;
        }

        public void write(StructureWriter outs) throws IOException
        {
            outs.writeString(point.label);
            outs.writeDoubles(point.coords);
            outs.writeString(cat.name());
            outs.writeString(action);
        }

        public void read(StructureReader ins) throws IOException
        {
            String label = ins.readString();
            double[] coords = ins.readDoubles();
            point = new CPoint(label, coords);

            cat = FeatureCategory.valueOf(ins.readString());

            action = ins.readString();
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
		synchronized (stateLock) {
			classifications = classifier.classify(features);
		}
		return classifications;
	}

	public void addDataPoint(FeatureCategory cat, ArrayList<Double> features, String label){
		IClassifier classifier = classifiers.get(cat);
		synchronized(stateLock){
            CPoint point = new CPoint(label, features);
            StackEntry entry = new StackEntry(point, cat, "ADD");
			classifier.add(point);
            undoStack.add(entry);
		}
	}

	public void clearData(){
		for(IClassifier classifier : classifiers.values()){
			synchronized(stateLock){
				classifier.clearData();
			}
		}
	}

	public void reloadData(){
		for(IClassifier classifier : classifiers.values()){
			synchronized(stateLock){
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
        synchronized (stateLock) {
            if (undoStack.size() < 1)
                return;
            StackEntry entry = undoStack.pollLast();

            if (entry.action.equals("ADD")) {
                classifiers.get(entry.cat).removeLast();
                redoStack.add(entry);
            } else {
                System.err.println("ERR: Unhandled undo case - "+entry.action);
            }
        }
    }

    /** Redo function. Takes the last undone action and redoes it */
    public void redo()
    {
        synchronized (stateLock) {
            if (redoStack.size() < 1)
                return;
            StackEntry entry = redoStack.pollLast();

            if (entry.action.equals("ADD")) {
                classifiers.get(entry.cat).add(entry.point);
                undoStack.add(entry);
            } else {
                System.err.println("ERR: Unhandled redo case - "+entry.action);
            }
        }
    }

    // XXX Might want to spawn a backup thread to do this...
    /** Write out a backup file of our current state. */
    public void writeState(String filename) throws IOException
    {
        synchronized (stateLock) {
            // As it stands, the undo/redo stacks possess all of the
            // information necessary to back up the entire system
            // state. Thus, the implementation of undo and redo
            // are directly connected with our ability to load from
            // this state
            StructureWriter outs = new TextStructureWriter(new BufferedWriter(new FileWriter(filename)));

            outs.writeString("undo");
            outs.writeInt(undoStack.size());
            outs.blockBegin();
            for (StackEntry entry: undoStack) {
                outs.blockBegin();
                entry.write(outs);
                outs.blockEnd();
            }
            outs.blockEnd();

            outs.writeString("redo");
            outs.writeInt(redoStack.size());
            outs.blockBegin();
            for (StackEntry entry: redoStack) {
                outs.blockBegin();
                entry.write(outs);
                outs.blockEnd();
            }
            outs.blockEnd();

            outs.close();
        }
    }

    /** Read in a backup file of our current state,
     *  resetting all state to match that of the file
     */
    public void readState(String filename) throws IOException
    {
        synchronized (stateLock) {
            // Reset state
            clearData();

            // Again, based on the premise than undo and redo work a certain way
            StructureReader ins = new TextStructureReader(new BufferedReader(new FileReader(filename)));

            String undoString = ins.readString();
            assert (undoString.equals("undo"));
            int undoSize = ins.readInt();
            ins.blockBegin();
            for (int i = 0; i < undoSize; i++) {
                ins.blockBegin();
                StackEntry entry = new StackEntry();
                entry.read(ins);

                if (entry.action.equals("ADD")) {
                    classifiers.get(entry.cat).add(entry.point);
                }

                undoStack.add(entry);

                ins.blockEnd();
            }
            ins.blockEnd();

            String redoString = ins.readString();
            assert (redoString.equals("redo"));
            int redoSize = ins.readInt();
            ins.blockBegin();
            for (int i = 0; i < redoSize; i++) {
                ins.blockBegin();
                StackEntry entry = new StackEntry();
                entry.read(ins);

                redoStack.add(entry);

                ins.blockEnd();
            }
            ins.blockEnd();

            ins.close();
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
