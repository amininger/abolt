package abolt.bolt;

import april.config.*;
import april.sim.SimObject;
import april.sim.SimWorld;
import april.sim.Simulator;
import april.util.*;
import april.vis.*;
import april.jmat.*;
import april.jmat.geom.GRay3D;
import lcm.lcm.*;

import abolt.lcmtypes.*;
import abolt.sim.BoltSim;
import abolt.vis.SelectionAnimation;
import abolt.kinect.*;
import abolt.classify.*;
import abolt.classify.Features.FeatureCategory;

import java.io.*;

import javax.swing.*;

import java.util.*;
import java.awt.BorderLayout;
import java.awt.Color;
import java.awt.Rectangle;
import java.awt.event.*;
import java.awt.image.*;

import abolt.arm.*;

public class Bolt extends JFrame implements LCMSubscriber
{    
    private IObjectManager objectManager;
    private ClassifierManager classifierManager;    
    
    // objects for visualization
    private WorldSimulator simulator;
    private JMenuItem clearData, reloadData;

    // LCM
    static LCM lcm = LCM.getSingleton();

    // Opts/Config
    GetOpt opts;
    Config config;

	// Sim stuff
    Simulator editor;
    SimWorld world;


    public Bolt(GetOpt opts_)
    {
        super("BOLT");
       
        
        this.setSize(800, 600);
        this.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);

        // Handle Options
        opts = opts_;
        try {
            config = new ConfigFile(opts.getString("config"));
        } catch (IOException ioex) {
            System.err.println("ERR: Could not load configuration from file");
            ioex.printStackTrace();
        }

        // Load calibration
        try {
            KUtils.loadCalibFromConfig(new ConfigFile(config.requireString("calibration.filepath")));
        } catch (IOException ioex) {
            System.err.println("ERR: Could not load calibration from file");
            ioex.printStackTrace();
        }

        classifierManager = new ClassifierManager(config);
        objectManager = new WorldObjectManager(classifierManager);
        

        setupMenuBar();
    	simulator = new WorldSimulator(opts);
    	this.add(simulator.getVisCanvas());

        // Subscribe to LCM
        lcm.subscribe("TRAINING_DATA", this);

        // TODO: arm stuff here
       //BoltArmCommandInterpreter interpreter = new BoltArmCommandInterpreter(segmenter, opts.getBoolean("debug"));

        this.setVisible(true);
    }
   
    
    private void setupMenuBar(){
    	JMenuBar menuBar = new JMenuBar();
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
        this.setJMenuBar(menuBar);
    }
    
    
    public IObjectManager getObjectManager(){
    	return objectManager;
    }
    
    public ClassifierManager getClassifierManager(){
    	return classifierManager;
    }

    @Override
    public void messageReceived(LCM lcm, String channel, LCMDataInputStream ins)
    {
       if(channel.equals("TRAINING_DATA")){
            try{
                training_data_t training = new training_data_t(ins);

                for(int i=0; i<training.num_labels; i++){
                    training_label_t tl = training.labels[i];
                    BoltObject obj = objectManager.getObjects().get(tl.id);
                    if(obj != null){
                        FeatureCategory cat = Features.getFeatureCategory(tl.cat.cat);
                        classifierManager.addDataPoint(cat, obj.getFeatures(cat), tl.label);
                    }
                }
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
        obs.click_id = objectManager.getSelectedId();
        obs.sensables = new String[0];
        obs.nsens = obs.sensables.length;
        obs.observations = objectManager.getObjectData();
        obs.nobs = obs.observations.length;

        lcm.publish("OBSERVATIONS",obs);
    }


    public static void main(String args[])
    {
        GetOpt opts = new GetOpt();

        opts.addBoolean('h', "help", false, "Show this help screen");
        opts.addString('c', "config", null, "Specify the configuration file");
        opts.addBoolean('d', "debug", false, "Toggle debugging mode");
        opts.addString('w', "world", "", "World file");
        opts.addInt('\0', "fps", 10, "Maximum frame rate");
        opts.addBoolean('\0', "start", false, "Start simulation automatically");

        if (!opts.parse(args)) {
            System.err.println("ERR: GetOpt - "+opts.getReason());
            return;
        }

        if (opts.getBoolean("help") || opts.getString("config") == null) {
            System.out.println("Usage: Must specify a configuration file");
            opts.doHelp();
            return;
        }

        KUtils.createDepthMap();
        Bolt bolt = new Bolt(opts);
    }
}


