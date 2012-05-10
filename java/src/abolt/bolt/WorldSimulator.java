package abolt.bolt;

import java.awt.event.MouseEvent;
import java.io.IOException;
import java.io.PrintStream;
import java.util.ArrayList;

import abolt.sim.SimBoltObject;
import abolt.sim.SimSensable;
import abolt.vis.SelectionAnimation;
import april.config.Config;
import april.jmat.LinAlg;
import april.jmat.geom.GRay3D;
import april.sim.Collisions;
import april.sim.SimObject;
import april.sim.SimWorld;
import april.util.EnvUtil;
import april.util.GetOpt;
import april.util.Tic;
import april.util.TimeUtil;
import april.vis.VisCanvas;
import april.vis.VisConsole;
import april.vis.VisEventAdapter;
import april.vis.VisLayer;
import april.vis.VisWorld;

public class WorldSimulator implements VisConsole.Listener{

	// Sim stuff
    SimWorld world;

    // Vis stuff
    VisWorld vw;
    VisCanvas vc;
    VisLayer vl;
    VisConsole console;

    // State
    Object selectionLock = new Object();
    SimObject selectedObject = null;
    SelectionAnimation animation = null;
    
    public WorldSimulator(GetOpt opts) {
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
	}
    
    public SimWorld getWorld(){
    	return world;
    }
    
    public VisCanvas getVisCanvas(){
    	return vc;
    }
    
    public SimObject getSelectedObject(){
    	return selectedObject;
    }
    
    private void loadWorld(GetOpt opts){
        try {
            if (opts.getString("world").length() > 0) {
                String worldFilePath = EnvUtil.expandVariables(opts.getString("world"));
                world = new SimWorld(worldFilePath, new Config());
            } else {
                world = new SimWorld(new Config());
            }

        } catch (IOException ex) {
            System.out.println("ex: "+ex);
            ex.printStackTrace();
            return;
        }
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
                for (SimObject obj : world.objects) {

                    double d = Collisions.collisionDistance(ray.getSource(), ray.getDir(), obj.getShape(), obj.getPose());

                    boolean validInstance = false;
                    validInstance |= obj instanceof SimSensable;
                    validInstance |= obj instanceof SimBoltObject;

                    if (d < bestd && validInstance) {
                        synchronized (selectionLock) {
                            selectedObject = obj;
                            bestd = d;
                        }
                    }
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

                TimeUtil.sleep(1000/fps);
            }
        }
    }
}
