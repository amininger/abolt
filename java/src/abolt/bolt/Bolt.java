package abolt.bolt;

import april.config.*;
import april.jmat.LinAlg;
import april.sim.Collisions;
import april.sim.Shape;
import april.sim.SphereShape;
import april.util.*;
import lcm.lcm.*;

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
	public static BoltObjectManager getObjectManager(){
		if(boltInstance == null){
			return null;
		}
		return boltInstance.objectManager;
	}
	public static SensableManager getSensableManager(){
		if(boltInstance == null){
			return null;
		}
		return boltInstance.sensableManager;
	}
	public static ClassifierManager getClassifierManager(){
		if(boltInstance == null){
			return null;
		}
		return boltInstance.classifierManager;
	}
	public static BoltSimulator getSimulator(){
		if(boltInstance == null){
			return null;
		}
		return boltInstance.simulator;
	}
	public static IBoltCamera getCamera(){
		if(boltInstance == null){
			return null;
		}
		return boltInstance.camera;
	}
	public static Segment getSegment(){
		if(boltInstance == null || !(boltInstance.camera instanceof KinectCamera)){
			return null;
		}
		return ((KinectCamera)boltInstance.camera).getSegment();
	}
	
    private BoltObjectManager objectManager;
    private SensableManager sensableManager;
    private ClassifierManager classifierManager; 
    private IBoltCamera camera;
    
    // objects for visualization
    private BoltSimulator simulator;
    private JMenuBar menuBar;
    private JMenuItem clearData, reloadData;
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
        

        setupMenuBar();
        
        classifierManager = new ClassifierManager(config);
        sensableManager = new SensableManager();
    	objectManager = new BoltObjectManager();
        
        if(opts.getBoolean("kinect")){
        	camera = new KinectCamera();        		
            BoltArmCommandInterpreter interpreter = new BoltArmCommandInterpreter(getSegment(), opts.getBoolean("debug"));
        } else {
        	// All done in simulation
        	camera = new SimKinect(400, 300);
        	armSimulator = new ArmSimulator();
        }

        simulator = new BoltSimulator(opts, menuBar);
    	this.add(simulator.getCanvas());
    	this.setJMenuBar(menuBar);

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
   
    
    private void setupMenuBar(){
    	menuBar = new JMenuBar();
        JMenu controlMenu = new JMenu("Control");
        menuBar.add(controlMenu);

        // Remove all data (no built in info)
        clearData = new JMenuItem("Clear All Data");
        clearData.addActionListener(new ActionListener(){
                @Override
                public void actionPerformed(ActionEvent e) {
                    System.out.println("CLEARED DATA");
                    classifierManager.clearData();
                }
            });
        controlMenu.add(clearData);

        // Remove all data (including training)
        reloadData = new JMenuItem("Reload Data");
        reloadData.addActionListener(new ActionListener(){
                @Override
                public void actionPerformed(ActionEvent e) {
                    classifierManager.reloadData();
                }
            });
        controlMenu.add(reloadData);
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
        opts.addBoolean('\0', "seg", false, "Show the segmentation instead of the simulator");
        opts.addString('w', "world", "", "World file");
        opts.addString('s', "sim-config", "", "Configuration file for the Simulator");
        opts.addInt('\0', "fps", 10, "Maximum frame rate");

        if (!opts.parse(args) || opts.getBoolean("help") || opts.getExtraArgs().size() > 0) {
            opts.doHelp();
            return;
        }

        if (opts.getString("config") == null) {
            System.out.println("Usage: Must specify a configuration file");
            opts.doHelp();
            return;
        }

        KUtils.createDepthMap();
        new Bolt(opts);
    }
}


