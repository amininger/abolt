package abolt.bolt;

import java.awt.Color;
import java.awt.event.MouseEvent;
import java.io.IOException;
import java.io.PrintStream;
import java.util.ArrayList;
import java.util.HashMap;

import abolt.classify.ConfidenceLabel;
import abolt.classify.Features.FeatureCategory;
import abolt.objects.BoltObject;
import abolt.objects.SimBoltObject;
import abolt.sim.SimSensable;
import abolt.vis.SelectionAnimation;
import april.config.Config;
import april.config.ConfigFile;
import april.jmat.LinAlg;
import april.jmat.geom.GRay3D;
import april.sim.Collisions;
import april.sim.SimObject;
import april.sim.SimWorld;
import april.sim.Simulator;
import april.util.EnvUtil;
import april.util.GetOpt;
import april.util.Tic;
import april.util.TimeUtil;
import april.vis.VisCameraManager.CameraPosition;
import april.vis.VisCanvas;
import april.vis.VisChain;
import april.vis.VisConsole;
import april.vis.VisEventAdapter;
import april.vis.VisLayer;
import april.vis.VisObject;
import april.vis.VisWorld;
import april.vis.VzLines;
import april.vis.VzMesh;
import april.vis.VzRectangle;
import april.vis.VzText;

public class BoltSimulator implements VisConsole.Listener, IBoltGUI{

	// Sim stuff
    SimWorld world;
    Simulator sim;

    // Vis stuff
    VisWorld vw;
    VisCanvas vc;
    VisLayer vl;
    VisConsole console;

    // State
    Object selectionLock = new Object();
    BoltObject selectedObject = null;
    SelectionAnimation animation = null;
    
    public BoltSimulator(GetOpt opts) {    	
		vw = new VisWorld();
		vl = new VisLayer(vw);
	    vc = new VisCanvas(vl);
	    vc.setTargetFPS(opts.getInt("fps"));
	    
	    console = new VisConsole(vw, vl, vc);
	    console.addListener(this);
	        
	    // Customized event handling
	    vl.addEventHandler(new BoltEventHandler());

        // Render updates about the world
        RenderThread rt = new RenderThread();
        rt.start();

        loadWorld(opts);
        sim = new Simulator(vw, vl, console, world);
	}
    
    public SimWorld getWorld(){
    	return world;
    }
    
    @Override
    public VisCanvas getCanvas(){
    	return vc;
    }
    
    @Override
    public BoltObject getSelectedObject(){
    	return selectedObject;
    }
    
    private void loadWorld(GetOpt opts){
    	try {
            Config config = new Config();
            if (opts.wasSpecified("sim-config"))
                config = new ConfigFile(EnvUtil.expandVariables(opts.getString("sim-config")));

            if (opts.getString("world").length() > 0) {
                String worldFilePath = EnvUtil.expandVariables(opts.getString("world"));
                world = new SimWorld(worldFilePath, config);
            } else {
                world = new SimWorld(config);
            }
        } catch (IOException ex) {
            System.out.println("ex: "+ex);
            ex.printStackTrace();
            return;
        }
    	world.setRunning(true);
    }

	public boolean consoleCommand(VisConsole console, PrintStream out, String command)
    {
        return false;
    }

	public ArrayList<String> consoleCompletions(VisConsole console, String prefix)
    {
        return null;    // Only using start and stop from sim, still
    }

    class BoltEventHandler extends VisEventAdapter
    {
        public int getDispatchOrder()
        {
            return -20; // Want to beat out the sim, though pass events through
        }

        /** Keep track of the last object to be selected in the sim world. */
        public boolean mousePressed(VisCanvas vc, VisLayer vl, VisCanvas.RenderInfo rinfo, GRay3D ray, MouseEvent e)
        {
            double bestd = Double.MAX_VALUE;

            synchronized(world) {
                HashMap<Integer, BoltObject> objects = Bolt.getObjectManager().getObjects();
                synchronized(objects){
                	for (BoltObject obj : objects.values()) {
                        double d = Collisions.collisionDistance(ray.getSource(), ray.getDir(), obj.getShape(), obj.getPose());

                        if (d < bestd) {
                            synchronized (selectionLock) {
                                selectedObject = obj;
                                bestd = d;
                            }
                        }
                    }
                }
                
                if(bestd == Double.MAX_VALUE){
                	HashMap<Integer, SimSensable> sensables = Bolt.getSensableManager().getSensables();
	                synchronized(sensables){
	                	for (SimSensable sens : sensables.values()) {
	                		if(!(sens instanceof BoltObject)){
	                			continue;
	                		}
	                		BoltObject obj = (BoltObject)sens;
	                        double d = Collisions.collisionDistance(ray.getSource(), ray.getDir(), obj.getShape(), obj.getPose());
	
	                        if (d < bestd) {
	                            synchronized (selectionLock) {
	                                selectedObject = (BoltObject)obj;
	                                bestd = d;
	                            }
	                        }
	                    }
	                }
                }
                
                if(selectedObject != null){
                	System.out.println("CLICKED: " + selectedObject.getID());
                }
            }

            if (bestd != Double.MAX_VALUE) {
                // Update animation position/state
                double[] xyz = LinAlg.resize(LinAlg.matrixToXyzrpy(selectedObject.getPose()), 3);
                double br = Math.abs(selectedObject.getShape().getBoundingRadius());
                animation = new SelectionAnimation(xyz, br*2);
            }


            return false;
        }
    }

    /** Render BOLT-specific content. */
    class RenderThread extends Thread
    {
        int fps = 20;

        public void run()
        {
            Tic tic = new Tic();
            while (true) {
                double dt = tic.toctic();
                if (animation != null) {
                    VisWorld.Buffer vb = vw.getBuffer("selection");
                    animation.step(dt);
                    vb.addBack(animation);
                    vb.swap();
                }

            	CameraPosition camera = vl.cameraManager.getCameraTarget();
        		double[] forward = LinAlg.normalize(LinAlg.subtract(camera.eye, camera.lookat));
        		// Spherical coordinates
                double psi = Math.PI/2.0 - Math.asin(forward[2]);   // psi = Pi/2 - asin(z)
                double theta = Math.atan2(forward[1], forward[0]);  // theta = atan(y/x)
                if(forward[0] == 0 && forward[1] == 0){
                	theta = -Math.PI/2;
                }
                double[][] tilt = LinAlg.rotateX(psi); 				// tilt up or down to face the camera vertically
                double[][] rot = LinAlg.rotateZ(theta + Math.PI/2); // rotate flat to face the camera horizontally
                double[][] faceCamera = LinAlg.matrixAB(rot, tilt);
                VisWorld.Buffer textBuffer = vw.getBuffer("textbuffer");
                
                HashMap<Integer, BoltObject> objects = Bolt.getObjectManager().getObjects();
                synchronized(objects){
       
                    for(BoltObject obj : objects.values()){
                    	String labelString = "";

                		String tf="<<monospaced,black,dropshadow=false>>";
                		labelString += String.format("%s%d\n", tf, obj.getID());
                		for(FeatureCategory cat : FeatureCategory.values()){
                			ConfidenceLabel label = obj.getLabels().getBestLabel(cat);
                    		labelString += String.format("%s%s:%.2f\n", tf, label.getLabel(), label.getConfidence());
                		}
                		VzText text = new VzText(labelString);
                		double[] textLoc = new double[]{obj.getPos()[0], obj.getPos()[1], obj.getPos()[2] + .1};
                        textBuffer.addBack(new VisChain(LinAlg.translate(textLoc), faceCamera, LinAlg.scale(.002), text));
                    }
                }
                textBuffer.swap();
                
                TimeUtil.sleep(1000/fps);
            }
            
        }
    }

    @Override
	public void drawObjects(HashMap<Integer, BoltObject> objects) {
    	synchronized(objects){
    		VisWorld.Buffer objectBuffer = vw.getBuffer("objects");
    		for(BoltObject obj : objects.values()){
    			objectBuffer.addBack(obj.getVisObject());
    		}
    		objectBuffer.swap();	
    	}			
	}
    
    @Override
    public void drawVisObjects(String bufferName, ArrayList<VisObject> objects){
    	VisWorld.Buffer buffer = vw.getBuffer(bufferName);
    	for(VisObject obj : objects){
    		buffer.addBack(obj);
    	}	
    	buffer.swap();
    }
}
