package abolt.classify;

import java.text.SimpleDateFormat;
import java.util.*;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.io.*;

import javax.swing.JMenu;
import javax.swing.JMenuItem;

import april.config.*;
import april.util.*;

import abolt.bolt.BoltObject;
import abolt.classify.Features.FeatureCategory;
import abolt.lcmtypes.*;

/**
 * @author aaron
 * Creates the classifiers used in the system and acts as a point of contact to them
 */
public class ClassifierManager{

    // Singleton handling
    private static ClassifierManager singleton = null;
    public static ClassifierManager getSingleton()
    {
        if (singleton == null) {
            singleton = new ClassifierManager();
        }
        return singleton;
    }
    
    public static void Initialize(Config config){
    	singleton = new ClassifierManager();
    	singleton.addClassifiers(config);
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
	
    PeriodicTasks tasks = new PeriodicTasks(2);   // Only one thread for now
    private static final double BACKUP_RATE = 10.0;

    public ClassifierManager()
    {
        init();
    }

	public ClassifierManager(Config config)
    {
        addClassifiers(config);
        init();
        
	}
	
	private void init(){
		tasks.addFixedDelay(new AutoSaveTask() , BACKUP_RATE);
    	tasks.setRunning(true);
	}
    
	/** 
	 * Creates a menu for options involving the classifiers
	 * 
	 * @return A menu for classifier functionality
	 */
    public JMenu generateMenu(){
    	return new ClassifierMenu();
    }
    
    /**
     * Creates classifiers according to the specifications in the given config file
     * <p>
     * Expects 3 files to be specified containing training data:
     * <ul>
     * <li>training.color_data</li>
     * <li>training.shape_data</li>
     * <li>training.size_data</li>
     * </ul>
     * 
     * @param config The config file from while the parameters are taken
     */
    public void addClassifiers(Config config)
    {
        String colorDataFile = null, shapeDataFile = null, sizeDataFile = null;
        colorDataFile = config.getString("training.color_data");
        shapeDataFile = config.getString("training.shape_data");
        //sizeDataFile = config.getString("training.size_data");

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
    
    /**
     * Adds a single custom classifier to the manager
     * 
     * @param cat The category of the classifier
     * @param classifier The classifier itself
     */
    public void addClassifier(FeatureCategory cat, IClassifier classifier){
    	classifiers.put(cat, classifier);
    }

    /**
     * Adds a custom data point to the classifiers 
     * 
     * @param cat The feature category to use
     * @param features A representation of the features (array of doubles)
     * @param label The label of the given data point
     */
	public void addDataPoint(FeatureCategory cat, ArrayList<Double> features, String label){
		IClassifier classifier = classifiers.get(cat);
		synchronized(stateLock){
            CPoint point = new CPoint(label, features);
            StackEntry entry = new StackEntry(point, cat, "ADD");
			classifier.add(point);
            undoStack.add(entry);
		}
	}

	public Classifications classify(FeatureCategory cat, BoltObject obj){
		IClassifier classifier = classifiers.get(cat);
		ArrayList<Double> features = obj.getFeatures(cat);
		if(features == null || classifier == null){
			return null;
		}
        	Classifications classifications;
		synchronized (stateLock) {
			classifications = classifier.classify(features);
		}
		return classifications;
	}
    
    public object_data_t getObjectData(BoltObject bo){
    	object_data_t objData = new object_data_t();
    	objData.utime = TimeUtil.utime();
    	objData.id = bo.getID();
    	objData.pos = bo.getPos();
    	objData.bbox = bo.getBBox();
    	objData.labels = bo.getLabels();
    	objData.num_labels = objData.labels.length;
    	ArrayList<categorized_data_t> cat_dats = new ArrayList<categorized_data_t>();
        int j = 0;
        for (FeatureCategory fc: classifiers.keySet()) {
            ArrayList<Double> features = bo.getFeatures(fc);
            categorized_data_t cat_dat = getCategorizedData(fc, features);
            if(cat_dat != null){
                cat_dats.add(cat_dat);
            }
        }

        objData.num_cat = cat_dats.size();
        objData.cat_dat = cat_dats.toArray(new categorized_data_t[cat_dats.size()]);
        return objData;
    }

	private void clearData(){
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

    private boolean hasUndo()
    {
        return undoStack.size() > 0;
    }

    private boolean hasRedo()
    {
        return redoStack.size() > 0;
    }

    /** Undo function. Undoes the last action taken by the user */
    private void undo()
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
    private void redo()
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
    private void writeState(String filename) throws IOException
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
    
    private categorized_data_t getCategorizedData(FeatureCategory fc, ArrayList<Double> features){
        IClassifier classifier = classifiers.get(fc);
        if(features == null || classifier == null){
        	return null;
        }
    	categorized_data_t cat_dat = new categorized_data_t();
        cat_dat = new categorized_data_t();
        cat_dat.cat = new category_t();
        cat_dat.cat.cat = Features.getLCMCategory(fc);
        // XXX: AM: I don't like this, but I don't want to deal with features from visual properties at the moment
        if(fc == FeatureCategory.WEIGHT || fc == FeatureCategory.SQUISHINESS){
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
        return cat_dat;
    }
    


    /** AutoSave the classifier state */
    class AutoSaveTask implements PeriodicTasks.Task
    {
        String filename;

        public AutoSaveTask()
        {
            Date date = new Date(System.currentTimeMillis());
            SimpleDateFormat sdf = new SimpleDateFormat("yyyy_MM_dd-HH_mm_ss");
            filename = "/tmp/bolt_autosave_"+sdf.format(date);
        }

        public void run(double dt)
        {
            try {
                writeState(filename);
            } catch (IOException ioex)  {
                System.err.println("ERR: Could not save to autosave file");
                ioex.printStackTrace();
            }
        }
    }
    
    private class ClassifierMenu extends JMenu{
    	JMenuItem clearData;
    	JMenuItem reloadData;
    	JMenuItem undoAction;
    	JMenuItem redoAction;
    	
        // Periodic tasks
        PeriodicTasks menuTasks = new PeriodicTasks(2);   // Only one thread for now
        
        private static final double REFRESH_RATE = .2;
    	
    	public ClassifierMenu(){
    		super("Classifiers");
    		init();
    		
    		menuTasks.addFixedDelay(new MenuUpdateTask(), REFRESH_RATE);
    		menuTasks.setRunning(true);
    	}
    	
    	public void init(){
            // Remove all data (no built in info)
            clearData = new JMenuItem("Clear All Data");
            clearData.addActionListener(new ActionListener(){
                    @Override
                    public void actionPerformed(ActionEvent e) {
                        System.out.println("CLEARED DATA");
                        clearData();
                    }
                });
            add(clearData);

            // Remove all data (including training)
            reloadData = new JMenuItem("Reload Data");
            reloadData.addActionListener(new ActionListener(){
                    @Override
                    public void actionPerformed(ActionEvent e) {
                        reloadData();
                    }
                });
            add(reloadData);

            // Undo & redo actions
            undoAction = new JMenuItem("Undo");
            undoAction.addActionListener(new ActionListener()
                {
                    public void actionPerformed(ActionEvent e) {
                        undo();
                    }
                });
            add(undoAction);

            redoAction = new JMenuItem("Redo");
            redoAction.addActionListener(new ActionListener()
                {
                    public void actionPerformed(ActionEvent e) {
                        redo();
                    }
                });
            add(redoAction);
    	}

        /** Update the status of menu entries to ensure only the
         *  appropriate ones are marked as active
         */
        class MenuUpdateTask implements PeriodicTasks.Task
        {
            public MenuUpdateTask() {}

            public void run(double dt)
            {
                if (hasUndo()) {
                    undoAction.setEnabled(true);
                } else {
                    undoAction.setEnabled(false);
                }

                if (hasRedo()) {
                    redoAction.setEnabled(true);
                } else {
                    redoAction.setEnabled(false);
                }
            }
        }
    }
}
