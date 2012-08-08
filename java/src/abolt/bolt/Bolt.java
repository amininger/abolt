package abolt.bolt;

import java.io.*;
import javax.swing.*;
import java.util.*;
import java.util.Timer;
import java.awt.event.*;
import java.text.*;

import lcm.lcm.*;

import april.config.*;
import april.jmat.LinAlg;
import april.sim.*;
import april.util.*;

import abolt.arm.*;
import abolt.lcmtypes.*;
import abolt.objects.BoltObject;
import abolt.objects.BoltObjectManager;
import abolt.objects.SensableManager;
import abolt.kinect.*;
import abolt.arm.ArmSimulator;
import abolt.arm.BoltArmCommandInterpreter;
import abolt.classify.*;
import abolt.classify.Features.FeatureCategory;


public class Bolt extends JFrame implements LCMSubscriber
{
	private static Bolt boltInstance;
	public static Bolt getSingleton(){
		return boltInstance;
	}

	public static IBoltCamera getCamera(){
		if(boltInstance == null){
			return null;
		}
		return boltInstance.camera;
	}

    private BoltObjectManager objectManager;
    private SensableManager sensableManager;
    private ClassifierManager classifierManager;
    private IBoltCamera camera;
    private KinectView kinectView;

    // objects for visualization
    private BoltSimulator simulator;
    private ArmSimulator armSimulator;

    // LCM
    static LCM lcm = LCM.getSingleton();
    private Timer sendObservationTimer;
    private static final int OBSERVATION_RATE = 4; // # sent per second

    // Periodic tasks
    PeriodicTasks tasks = new PeriodicTasks(2);   // Only one thread for now

    // GUI Stuff
    JMenuBar menuBar;
    JMenu controlMenu, editMenu;
    JMenuItem clearData, reloadData;
    JMenuItem undoAction, redoAction;

    public Bolt(GetOpt opts)
    {
        super("BOLT");

        boltInstance = this;
        this.setSize(800, 600);
        this.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);

        // Handle Options
        Config config;
        try {
            config = new ConfigFile(opts.getString("config"));
        } catch (IOException ioex) {
            System.err.println("ERR: Could not load configuration from file");
            ioex.printStackTrace();
            return;
        }

        // Load calibration
        try {
            KUtils.loadCalibFromConfig(new ConfigFile(config.requireString("calibration.filepath")));
        } catch (IOException ioex) {
            System.err.println("ERR: Could not load calibration from file");
            ioex.printStackTrace();
        }

        // Initialize classifier manager
        classifierManager = ClassifierManager.getSingleton();
        classifierManager.addClassifiers(config); // XXX Auto-coded config stuff here. Meh

        if (opts.getString("backup") != null) {
            try {
                System.out.println("ATTN: Loading from autosave file");
                classifierManager.readState(opts.getString("backup"));
                System.out.println("ATTN: Successfully restored from autosave file");
            } catch (IOException ioex) {
                System.err.println("ERR: Failure to load from autosave file");
                ioex.printStackTrace();
            }
        }

        // Initialize sensable manager
        sensableManager = SensableManager.getSingleton();

        // Initialize object manager
    	objectManager = BoltObjectManager.getSingleton();

        // If we specify that we would like to use an actual kinect,
        // initialize a kinect camera to process incoming kinect_status
        // messages. Otherwise, initialize a simulated kinect to generate
        // virtual point clouds based on the sim environment

    	// Initialize the simulator
        simulator = new BoltSimulator(opts);


        if(opts.getBoolean("kinect")){
        	camera = new KinectCamera(simulator.getVisWorld()); // Lauren
        	kinectView = new KinectView();
            // XXX We'd like to remove this middleman to the arm
            //BoltArmCommandInterpreter interpreter = new BoltArmCommandInterpreter(getSegment(), opts.getBoolean("debug"));
        } else {
        	camera = new SimKinect(400, 300, simulator);
        	armSimulator = new ArmSimulator(simulator);
        }

        // Initialize the JMenuBar
        createMenuBar();
        simulator.addToMenu(menuBar);
        this.setJMenuBar(menuBar);
    	this.add(simulator.getCanvas());

        // Subscribe to LCM
        lcm.subscribe("TRAINING_DATA", this);
        lcm.subscribe("ROBOT_COMMAND", this);


        // TODO: arm stuff here

        this.setVisible(true);
        class SendObservationTask extends TimerTask{
			public void run() {
        		sendMessage();
			}
        }
        sendObservationTimer = new Timer();
        sendObservationTimer.schedule(new SendObservationTask(), 1000, 1000/OBSERVATION_RATE);

        // Write to file task
        tasks.addFixedDelay(new AutoSaveTask(), 10.0);
        tasks.addFixedDelay(new MenuUpdateTask(), 0.2);
        tasks.setRunning(true);
    }


    public void createMenuBar()
    {
    	menuBar = new JMenuBar();
        controlMenu = new JMenu("Control");

        menuBar.add(controlMenu);

        // Remove all data (no built in info)
        clearData = new JMenuItem("Clear All Data");
        clearData.addActionListener(new ActionListener(){
                @Override
                public void actionPerformed(ActionEvent e) {
                    System.out.println("CLEARED DATA");
                    ClassifierManager.getSingleton().clearData();
                }
            });
        controlMenu.add(clearData);

        // Remove all data (including training)
        reloadData = new JMenuItem("Reload Data");
        reloadData.addActionListener(new ActionListener(){
                @Override
                public void actionPerformed(ActionEvent e) {
                    ClassifierManager.getSingleton().reloadData();
                }
            });
        controlMenu.add(reloadData);

        // Edit menu XXX
        editMenu = new JMenu("Edit");

        menuBar.add(editMenu);

        // Undo & redo actions
        undoAction = new JMenuItem("Undo");
        undoAction.addActionListener(new ActionListener()
            {
                public void actionPerformed(ActionEvent e) {
                    classifierManager.undo();
                }
            });
        editMenu.add(undoAction);

        redoAction = new JMenuItem("Redo");
        redoAction.addActionListener(new ActionListener()
            {
                public void actionPerformed(ActionEvent e) {
                    classifierManager.redo();
                }
            });
        editMenu.add(redoAction);
    }


    @Override
    public void messageReceived(LCM lcm, String channel, LCMDataInputStream ins)
    {
       if(channel.equals("TRAINING_DATA")){
            try{
                training_data_t training = new training_data_t(ins);

                for(int i=0; i<training.num_labels; i++){
                    training_label_t tl = training.labels[i];
                    BoltObject obj;
                    synchronized(objectManager.objects){
                        obj = objectManager.objects.get(tl.id);
                    }
                    if(obj != null){
                        FeatureCategory cat = Features.getFeatureCategory(tl.cat.cat);
                        ArrayList<Double> features = obj.getFeatures(cat);
                        if(features != null){
                            classifierManager.addDataPoint(cat, features, tl.label);
                        }
                    }
                }
            }catch (IOException e) {
                e.printStackTrace();
                return;
            }
        } else if(channel.equals("ROBOT_COMMAND")){
        	try{
        		robot_command_t command = new robot_command_t(ins);
        		sensableManager.performAction(command.action);
        	}catch (IOException e) {
                e.printStackTrace();
                return;
            }
        }
    }


    public void sendMessage()
    {
        observations_t obs = new observations_t();
        obs.utime = TimeUtil.utime();
        synchronized(objectManager.objects){
        	obs.click_id = simulator.getSelectedId();
        }
        obs.sensables = sensableManager.getSensableStrings();
        obs.nsens = obs.sensables.length;
        obs.observations = classifierManager.getObjectData();
        obs.nobs = obs.observations.length;

        lcm.publish("OBSERVATIONS",obs);
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
                classifierManager.writeState(filename);
            } catch (IOException ioex)  {
                System.err.println("ERR: Could not save to autosave file");
                ioex.printStackTrace();
            }
        }
    }

    /** Update the status of menu entries to ensure only the
     *  appropriate ones are marked as active
     */
    class MenuUpdateTask implements PeriodicTasks.Task
    {
        public MenuUpdateTask()
        {

        }

        public void run(double dt)
        {
            if (classifierManager.hasUndo()) {
                undoAction.setEnabled(true);
            } else {
                undoAction.setEnabled(false);
            }

            if (classifierManager.hasRedo()) {
                redoAction.setEnabled(true);
            } else {
                redoAction.setEnabled(false);
            }
        }
    }

    public static void main(String args[])
    {
        GetOpt opts = new GetOpt();

        // XXX Todo: clean up arguments
        opts.addBoolean('h', "help", false, "Show this help screen");
        opts.addString('c', "config", null, "Specify the configuration file for Bolt");
        opts.addBoolean('d', "debug", false, "Toggle debugging mode");
        opts.addBoolean('k', "kinect", false, "Use kinect data to create objects");
        opts.addBoolean('a', "arm", false, "Run with the actual arm");
        opts.addBoolean('\0', "seg", false, "Show the segmentation instead of the simulator");
        opts.addString('w', "world", null, "World file");
        opts.addString('s', "sim-config", null, "Configuration file for the Simulator");
        opts.addString('\0', "backup", null, "Load from backup file");
        opts.addInt('\0', "fps", 10, "Maximum frame rate");

        if (!opts.parse(args) || opts.getBoolean("help") || opts.getExtraArgs().size() > 0) {
            opts.doHelp();
            return;
        }

        Config config;
        if (opts.getString("config") == null) {
            System.out.println("Usage: Must specify a configuration file");
            opts.doHelp();
            return;
        } else {
            try {
                config = new ConfigFile(opts.getString("config"));
            } catch (IOException ioex) {
                System.err.println("ERR: "+ioex);
                ioex.printStackTrace();
                return;
            }
        }

        KUtils.createDepthMap();

        // Initialize the arm
        BoltArm.getSingleton().initArm(config);

        Bolt bolt = new Bolt(opts);

        BoltArmCommandInterpreter interpreter = new BoltArmCommandInterpreter(opts.getBoolean("debug"));
        BoltArmController controller = new BoltArmController();
        if (opts.getBoolean("arm")) {
            ArmDriver armDriver = new ArmDriver(config);
            (new Thread(armDriver)).start();
            if (opts.getBoolean("debug")) {
                BoltArmDemo demo = new BoltArmDemo(false);
            }
        } else {
            if (opts.getBoolean("debug")) {
                BoltArmDemo demo = new BoltArmDemo(true);
            }
        }

        if (opts.getBoolean("debug")) {
            ClassifyDebugGUI clDebugger = new ClassifyDebugGUI();
        }
    }
}


