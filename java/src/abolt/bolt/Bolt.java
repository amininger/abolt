package abolt.bolt;

import java.io.*;
import javax.swing.*;
import april.config.*;
import april.util.*;

import abolt.arm.*;
import abolt.kinect.*;
import abolt.arm.ArmSimulator;
import abolt.arm.BoltArmCommandInterpreter;
import abolt.classify.*;


public class Bolt extends JFrame
{
	private static Bolt boltInstance;
	public static Bolt getSingleton(){
		return boltInstance;
	}

    private KinectView kinectView;

    // objects for visualization
    private BoltSimulator simulator;
    private ArmSimulator armSimulator;

    // GUI Stuff
    JMenuBar menuBar;

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
        ClassifierManager.Initialize(config);
        if (opts.getString("backup") != null) {
            try {
                System.out.println("ATTN: Loading from autosave file");
                ClassifierManager.getSingleton().readState(opts.getString("backup"));
                System.out.println("ATTN: Successfully restored from autosave file");
            } catch (IOException ioex) {
                System.err.println("ERR: Failure to load from autosave file");
                ioex.printStackTrace();
            }
        }
        
        // Initialize LCM
        LCMReceiver.Initialize();
        LCMBroadcaster.Initialize();

        
    	// Initialize the simulator
        simulator = new BoltSimulator(opts);
        
        // Initialize the arm simulator (if needed)
        if(!opts.getBoolean("arm")){
        	ArmSimulator.Initialize(simulator);
        }

        Perception.Initialize();

        // Initialize the JMenuBar
        createMenuBar();
        simulator.addToMenu(menuBar);
        this.setJMenuBar(menuBar);
    	this.add(simulator.getCanvas());

        // TODO: arm stuff here
        this.setVisible(true);
        LCMBroadcaster.getSingleton().start();
    }


    public void createMenuBar()
    {
    	menuBar = new JMenuBar();
    	menuBar.add(ClassifierManager.getSingleton().generateMenu());
    }

    public static void main(String args[])
    {
        GetOpt opts = new GetOpt();

        // XXX Todo: clean up arguments
        opts.addBoolean('h', "help", false, "Show this help screen");
        opts.addString('c', "config", null, "Specify the configuration file for Bolt");
        opts.addBoolean('d', "debug", false, "Toggle debugging mode");
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
        //BoltArmController controller = new BoltArmController();
        if (opts.getBoolean("arm")) {
            BoltArmController controller = new BoltArmController();
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


