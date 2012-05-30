package abolt.bolt;

import java.awt.Color;
import java.awt.Rectangle;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.awt.event.MouseEvent;
import java.io.IOException;
import java.io.PrintStream;
import java.util.ArrayList;
import java.util.HashMap;

import javax.swing.*;

import abolt.classify.ConfidenceLabel;
import abolt.classify.Features.FeatureCategory;
import abolt.collision.ShapeToVisObject;
import abolt.kinect.KUtils;
import abolt.kinect.SimKinect;
import abolt.objects.BoltObject;
import abolt.objects.BoltObjectManager;
import abolt.objects.ISimBoltObject;
import abolt.sim.SimSensable;
import abolt.vis.SelectionAnimation;
import april.config.Config;
import april.config.ConfigFile;
import april.jmat.LinAlg;
import april.jmat.geom.GRay3D;
import april.sim.*;
import april.util.*;
import april.vis.*;
import april.vis.VisCameraManager.CameraPosition;
import april.vis.VzMesh.Style;

public class BoltSimulator implements VisConsole.Listener{

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
    
    enum ViewType{
    	POINT_CLOUD, SOAR, IMAGES, SIM_SHAPES
    };
    ViewType viewType;
    
    public BoltSimulator(GetOpt opts, JMenuBar menuBar) {    	
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
        viewType = ViewType.SOAR;
        
        setupMenu(menuBar);
	}
    
    public SimWorld getWorld(){
    	return world;
    }
    
    public VisCanvas getCanvas(){
    	return vc;
    }
    
    public VisLayer getLayer(){
    	return vl;
    }
    
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
    
    private void setupMenu(JMenuBar menuBar){
    	JMenu simMenu = new JMenu("Simulator");
    	ButtonGroup group = new ButtonGroup();
    	
    	JRadioButtonMenuItem pointView = new JRadioButtonMenuItem("Point Cloud View");
    	pointView.addActionListener(new ActionListener(){
			@Override
			public void actionPerformed(ActionEvent arg0) {
				setView(ViewType.POINT_CLOUD);
			}
    	});
    	group.add(pointView);
    	simMenu.add(pointView);
    	
    	JRadioButtonMenuItem soarView = new JRadioButtonMenuItem("Soar View");
    	soarView.addActionListener(new ActionListener(){
			@Override
			public void actionPerformed(ActionEvent arg0) {
				setView(ViewType.SOAR);
			}
    	});
    	group.add(soarView);
    	simMenu.add(soarView);

    	JRadioButtonMenuItem projView = new JRadioButtonMenuItem("Object Projection View");
    	projView.addActionListener(new ActionListener(){
			@Override
			public void actionPerformed(ActionEvent arg0) {
				setView(ViewType.IMAGES);
			}
    	});
    	//group.add(projView);
    	//simMenu.add(projView);
    	
    	JRadioButtonMenuItem simView = new JRadioButtonMenuItem("Normal Sim View");
    	simView.addActionListener(new ActionListener(){
			@Override
			public void actionPerformed(ActionEvent arg0) {
				setView(ViewType.SIM_SHAPES);
			}
    	});
    	group.add(simView);
    	simMenu.add(simView);
    	
    	menuBar.add(simMenu); 
    }
    
    private void setView(ViewType view){
    	this.viewType = view;
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
            	BoltObjectManager objManager = Bolt.getObjectManager();
                synchronized(objManager.objects){
                	for (BoltObject obj : objManager.objects.values()) {
                        double d = Collisions.collisionDistance(ray.getSource(), ray.getDir(), obj.getShape(), obj.getPoseMatrix());
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
	                        double d = Collisions.collisionDistance(ray.getSource(), ray.getDir(), obj.getShape(), obj.getPoseMatrix());
	
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
                double[] xyz = LinAlg.resize(LinAlg.matrixToXyzrpy(selectedObject.getPoseMatrix()), 3);
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
                
                BoltObjectManager objManager = Bolt.getObjectManager();
                synchronized(objManager.objects){
                    for(BoltObject obj : objManager.objects.values()){
                    	String labelString = "";

                		String tf="<<monospaced,black,dropshadow=false>>";
                		labelString += String.format("%s%d\n", tf, obj.getID());
                		for(FeatureCategory cat : FeatureCategory.values()){
                			ConfidenceLabel label = obj.getLabels().getBestLabel(cat);
                    		labelString += String.format("%s%s:%.2f\n", tf, label.getLabel(), label.getConfidence());
                		}
                		VzText text = new VzText(labelString);
                		double[] textLoc = new double[]{obj.getPose()[0], obj.getPose()[1], obj.getPose()[2] + .1};
                        textBuffer.addBack(new VisChain(LinAlg.translate(textLoc), faceCamera, LinAlg.scale(.002), text));
                    }
                }
                textBuffer.swap();
                
                TimeUtil.sleep(1000/fps);
            }
        }
    }
    
	public void drawObjects(HashMap<Integer, BoltObject> objects) {
    	synchronized(objects){
    		VisWorld.Buffer objectBuffer = vw.getBuffer("objects");
    		switch(viewType){
    		case POINT_CLOUD:
    			for(BoltObject obj : objects.values()){
    				ArrayList<double[]> points = obj.getInfo().points;
	    			if(points != null && points.size() > 0){
	        			VisColorData colors = new VisColorData();
	        			for(int i = 0; i < points.size(); i++){
	        				double[] pt = points.get(i);
	        				colors.add((int)pt[3]);
	        				points.set(i, Bolt.getCamera().getWorldCoords(pt));
	        			}
	        			VzPoints visPts = new VzPoints(new VisVertexData(points), new VzPoints.Style(colors, 2));
	        			objectBuffer.addBack(visPts);
	    			}
        		}
    			break;
    		case SOAR:
    			for(BoltObject obj : objects.values()){
    				objectBuffer.addBack(obj.getVisObject());
        		}
    			break;
    		case IMAGES:
    			CameraPosition camPos = Bolt.getSimulator().getLayer().cameraManager.getCameraTarget();
    			if(camPos != null){
    				double[][] view = camPos.getModelViewMatrix();
    				double[][] proj = camPos.getProjectionMatrix();
        			for(BoltObject obj : objects.values()){
                		VzImage img = new VzImage(obj.getInfo().getImage());
                		Rectangle bbox = obj.getInfo().getProjectedBBox();
                		//objectBuffer.addBack(new VisChain(LinAlg.translate(obj.getPose()[0], obj.getPose()[1], .01), invView, LinAlg.scale(.01, .01, .01), img));
            		}
    			}
    			break;
    		case SIM_SHAPES:
    			for(BoltObject obj : objects.values()){
    				if(obj.getInfo().createdFrom != null){
    					ISimBoltObject simObj = obj.getInfo().createdFrom;
    					ArrayList<VisObject> visObjs = ShapeToVisObject.getVisObjects(simObj.getAboltShape(), new VzMesh.Style(simObj.getColor()));
    					for(VisObject visObj : visObjs){
        					objectBuffer.addBack(new VisChain(simObj.getPose(), visObj));
    					}
    				}
        		}
    			break;
    		}
    		
    		objectBuffer.swap();	
    	}			
	}
    
    
    public void drawVisObjects(String bufferName, ArrayList<VisObject> objects){
    	VisWorld.Buffer buffer = vw.getBuffer(bufferName);
    	for(VisObject obj : objects){
    		buffer.addBack(obj);
    	}	
    	buffer.swap();
    }
}
