package abolt.bolt;

import april.config.*;
import april.jmat.LinAlg;
import april.sim.Collisions;
import april.sim.Shape;
import april.sim.SphereShape;
import april.util.*;
import lcm.lcm.*;

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

import java.io.*;

import javax.swing.*;

import java.util.*;
import java.util.Timer;
import java.awt.event.*;

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

    // objects for visualization
    private BoltSimulator simulator;
    private ArmSimulator armSimulator;

    // LCM
    static LCM lcm = LCM.getSingleton();
    private Timer sendObservationTimer;
    private static final int OBSERVATION_RATE = 2; // # sent per second

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
        classifierManager.addClassifiers(config);

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
        	camera = new KinectCamera();
            // XXX We'd like to remove this middleman to the arm
            //BoltArmCommandInterpreter interpreter = new BoltArmCommandInterpreter(getSegment(), opts.getBoolean("debug"));
        } else {
        	camera = new SimKinect(800, 600, simulator);
        	armSimulator = new ArmSimulator(simulator);
        }

        // Initialize the JMenuBar
        JMenuBar menuBar = createMenuBar();
        simulator.addToMenu(menuBar, opts.getBoolean("kinect"));
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
    }


    public static JMenuBar createMenuBar(){
    	JMenuBar menuBar = new JMenuBar();
        JMenu controlMenu = new JMenu("Control");
        JMenuItem clearData, reloadData;

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

        return menuBar;
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
        BoltObject selectedObj = simulator.getSelectedObject();
        if(selectedObj != null){
        	obs.click_id = selectedObj.getID();
        } else {
        	obs.click_id = -1;
        }
        obs.sensables = sensableManager.getSensableStrings();
        obs.nsens = obs.sensables.length;
        obs.observations = objectManager.getObjectData();
        obs.nobs = obs.observations.length;

        lcm.publish("OBSERVATIONS",obs);
    }


    public static void main(String args[])
    {
        GetOpt opts = new GetOpt();

        opts.addBoolean('h', "help", false, "Show this help screen");
        opts.addString('c', "config", null, "Specify the configuration file for Bolt");
        opts.addBoolean('d', "debug", false, "Toggle debugging mode");
        opts.addBoolean('k', "kinect", false, "Use kinect data to create objects");
        opts.addBoolean('a', "arm", false, "Run with the actual arm");
        opts.addBoolean('\0', "seg", false, "Show the segmentation instead of the simulator");
        opts.addString('w', "world", null, "World file");
        opts.addString('s', "sim-config", null, "Configuration file for the Simulator");
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
        Bolt bolt = new Bolt(opts);
        if (opts.getBoolean("arm")) {
            ArmDriver armDriver = new ArmDriver(config);
            (new Thread(armDriver)).start();
            BoltArmCommandInterpreter interpreter = new BoltArmCommandInterpreter(false);//opts.getBoolean("debug"));
            BoltArmController controller = new BoltArmController();
            if (opts.getBoolean("debug")) {
                BoltArmDemo demo = new BoltArmDemo(null); // XXX This won't quite make sense
            }
        }
    }
}


