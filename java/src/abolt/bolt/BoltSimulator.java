package abolt.bolt;

import java.awt.*;
import java.awt.event.*;
import java.io.IOException;
import java.io.PrintStream;
import java.util.*;

import javax.swing.*;

import abolt.kinect.*;
import abolt.classify.*;
import abolt.classify.Features.FeatureCategory;
import abolt.sim.SimLocation;
import abolt.vis.SelectionAnimation;
import april.config.*;
import april.jmat.*;
import april.jmat.geom.*;
import april.sim.*;
import april.sim.Shape;
import april.util.*;
import april.vis.*;

public class BoltSimulator implements VisConsole.Listener{
    private final static int K_HEIGHT = 480;
    private final static int K_WIDTH = 640;
    
    static BoltSimulator singleton = null;
    public static BoltSimulator getSingleton(){
    	return singleton;
    }

	// Sim stuff
    private SimWorld world;
    private Simulator sim;

    // Vis stuff
    private  VisWorld vw;
    private  VisCanvas vc;
    private VisLayer vl;
    private VisConsole console;

    // State
    private Object selectionLock = new Object();
    private int selectedId = -1;
    private  SelectionAnimation animation = null;
    
    private SimView simView;
    private SimClickHandler clickHandler;

    public BoltSimulator(GetOpt opts) {
    	singleton = this;
    	
		vw = new VisWorld();
		vl = new VisLayer(vw);
	    vc = new VisCanvas(vl);
	    vc.setTargetFPS(opts.getInt("fps"));

	    console = new VisConsole(vw, vl, vc);
	    console.addListener(this);

	    // Customized event handling
	    vl.addEventHandler(new BoltEventHandler());

        loadWorld(opts);
        sim = new Simulator(vw, vl, console, world);
        simView = new SimView.PointCloudView();
        clickHandler = new SimClickHandler.SelectObject();

        // Render updates about the world
        RenderThread rt = new RenderThread();
        rt.start();

	}

    public VisWorld.Buffer getVisWorld(){
        return vw.getBuffer("arm_cylinders");
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

    public int getSelectedId(){
    	return selectedId;
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

    public void addToMenu(JMenuBar menuBar){
    	JMenu simMenu = new JMenu("Simulator");

    	addViewTypeMenu(simMenu);
    	simMenu.addSeparator();
    	addClickTypeMenu(simMenu);

    	menuBar.add(simMenu);
    }

    private void addClickTypeMenu(JMenu menu){
    	menu.add(new JLabel("Change Selection Mode"));
    	ButtonGroup clickGroup = new ButtonGroup();

    	JRadioButtonMenuItem select = new JRadioButtonMenuItem("Select Object");
    	select.addActionListener(new ActionListener(){
			public void actionPerformed(ActionEvent arg0) {
				clickHandler = new SimClickHandler.SelectObject();
			}
    	});
    	clickGroup.add(select);
    	menu.add(select);

    	JRadioButtonMenuItem visiblity = new JRadioButtonMenuItem("Toggle Visibility");
    	visiblity.addActionListener(new ActionListener(){
			public void actionPerformed(ActionEvent arg0) {
				clickHandler = new SimClickHandler.ToggleVisibility();
			}
    	});
    	clickGroup.add(visiblity);
    	menu.add(visiblity);


    	JRadioButtonMenuItem changeId = new JRadioButtonMenuItem("Change Id");
    	changeId.addActionListener(new ActionListener(){
			public void actionPerformed(ActionEvent arg0) {
				clickHandler = new SimClickHandler.ChangeId();
			}
    	});
    	clickGroup.add(changeId);
    	menu.add(changeId);
    }

    private void addViewTypeMenu(JMenu menu){
    	menu.add(new JLabel("Change Simulator View"));
    	ButtonGroup viewGroup = new ButtonGroup();

    	JRadioButtonMenuItem pointView = new JRadioButtonMenuItem("Point Cloud View");
    	pointView.addActionListener(new ActionListener(){
			public void actionPerformed(ActionEvent arg0) {
				simView = new SimView.PointCloudView();
			}
    	});
    	viewGroup.add(pointView);
    	menu.add(pointView);

    	JRadioButtonMenuItem soarView = new JRadioButtonMenuItem("Soar View");
    	soarView.addActionListener(new ActionListener(){
			public void actionPerformed(ActionEvent arg0) {
				simView = new SimView.SoarView();
			}
    	});
    	viewGroup.add(soarView);
    	menu.add(soarView);
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
            	HashMap<Integer, BoltObject> objects = Perception.getSingleton().getCurrentObjects();
            	BoltObject selected = null;
            	for (BoltObject obj : objects.values()) {
            		double d;
            		double[][] poseMatrix = LinAlg.xyzrpyToMatrix(obj.getPos());
            		if(obj.sourceObject != null && obj.sourceObject.getAboltShape() != null){
                        d = abolt.collision.Collisions.collisionDistance(ray.getSource(), ray.getDir(), obj.sourceObject.getAboltShape(), poseMatrix);
            		} else {
                		Shape sphere = new SphereShape(obj.getRadius());
                        d = Collisions.collisionDistance(ray.getSource(), ray.getDir(), sphere, poseMatrix);
            		}
                    if (d < bestd) {
                        synchronized (selectionLock) {
                        	selected = obj;
                            bestd = d;
                        }
                    }
                }
            	if(selected != null){
            		changeSelected(clickHandler.clicked(selected));
            	}
            }

            return false;
        }
    }
    
    private void changeSelected(int id){
    	if(id == -1){
    		return;
    	}
    	animation = null;
    	selectedId = id;
    }

    /** Render BOLT-specific content. */
    class RenderThread extends Thread
    {
        int fps = 40;

        public void run()
        {
            Tic tic = new Tic();
            while (true) {
                TimeUtil.sleep(10);
                double dt = tic.toctic();
                if(Perception.getSingleton() == null){
                	continue;
                }

                // Object drawing
            	VisWorld.Buffer objBuffer = vw.getBuffer("objects");
                simView.draw(objBuffer);   // XXX meh
                objBuffer.swap();
            	
                // Draw Text Labels
                VisWorld.Buffer textBuffer = vw.getBuffer("textbuffer");
            	double[][] faceCamera = KUtils.getFaceCameraTransform(vl.cameraManager.getCameraTarget());
                drawTextLabels(textBuffer, faceCamera);
                textBuffer.swap();
                
                // Draw the selection animation
                VisWorld.Buffer selBuffer = vw.getBuffer("selection");
                drawSelectionAnimation(selBuffer, dt);
                selBuffer.swap();
                
                // Draw the dark rectangle
                VisWorld.Buffer tableBuffer = vw.getBuffer("table");
                tableBuffer.addBack(new VisChain(LinAlg.translate(new double[]{0, 0, .00001}), new VzRectangle(.95, .95, new VzMesh.Style(Color.gray))));
                tableBuffer.swap();

            }
        }
    }
	
	private void drawSelectionAnimation(VisWorld.Buffer buffer, double dt){
		HashMap<Integer, BoltObject> objects = Perception.getSingleton().getCurrentObjects();
		boolean isSelected = false;
		for(BoltObject obj : objects.values()){
			if(obj.getID() != selectedId){
				continue;
			}
			isSelected = true;
    		if(animation == null){
    			double[] animationCenter = LinAlg.add(obj.getPos(), new double[]{0, 0, .02, 0, 0, 0});
                animation = new SelectionAnimation(animationCenter, obj.getRadius()*1.2);
    		}
		}
		if(!isSelected){
			animation = null;
			buffer.clear();
		}
		if(animation != null){
			animation.step(dt);
			buffer.addBack(animation);
		}
	}
	
	private void drawTextLabels(VisWorld.Buffer buffer, double[][] faceCamera){
		HashMap<Integer, BoltObject> objects = Perception.getSingleton().getCurrentObjects();
        ClassifierManager clManager = ClassifierManager.getSingleton();
        for(BoltObject obj : objects.values()){
        	if(!obj.isVisible()){
        		continue;
        	}
        	if(obj.sourceObject != null && obj.sourceObject instanceof SimLocation){
        		continue;	// Don't print locations
        	}
        	String labelString = "";
    		String tf="<<monospaced,black,dropshadow=false>>";
    		labelString += String.format("%s%d\n", tf, obj.getID());
    		for(FeatureCategory cat : FeatureCategory.values()){
                Classifications cs = clManager.classify(cat, obj);
                if(cs == null){
                	continue;
                }
        		labelString += String.format("%s%s:%.2f\n", tf, cs.getBestLabel().label, cs.getBestLabel().weight);
    		}
    		VzText text = new VzText(labelString);
    		double[] textLoc = new double[]{obj.getPos()[0], obj.getPos()[1], obj.getPos()[2] + .1};
    		buffer.addBack(new VisChain(LinAlg.translate(textLoc), faceCamera, LinAlg.scale(.002), text));
        }
	}

    public void drawVisObjects(String bufferName, ArrayList<VisObject> objects){
    	VisWorld.Buffer buffer = vw.getBuffer(bufferName);
    	for(VisObject obj : objects){
    		buffer.addBack(obj);
    	}
    	buffer.swap();
    }


//	public void performAction(String action){
//		// Expecting the form 'ID=234,DOOR=OPEN'
//		String[] args = action.toUpperCase().split(",");
//		if(args.length < 2){
//			return;
//		}
//		String[] idArg = args[0].split("=");
//		if(idArg.length < 2 || !idArg[0].equals("ID")){
//			return;
//		}
//		int id = Integer.parseInt(idArg[1]);
//		for(BoltObject obj : currentObjects){
//			if(obj instanceof SimActionable){
//				
//			}
//		}
//		synchronized(sensables){
//			SimSensable sens = sensables.get(id);
//			if(sens == null || !(sens instanceof SimActionable)){
//				return;
//			}
//			((SimActionable)sens).setState(args[1]);
//		}
//	}
}
