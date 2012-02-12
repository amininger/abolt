package abolt.sim;

import java.awt.*;
import java.awt.event.*;
import javax.swing.*;
import java.io.*;
import java.util.*;

import april.config.*;
import april.jmat.*;
import april.jmat.geom.*;
import april.sim.*;
import april.util.*;
import april.vis.*;

import abolt.vis.*;

/** A variation of the core april-simulator with specific
 *  additions for BOLT. Gives the user access to the
 *  VisWorld to implement animations, etc. as desired as
 *  well as control the creation of simulated objects outside
 *  of the standard SimBox and SimBox. Just a wrapper around
 *  simulator! Not an extension, for now.
 */
public class BoltSim
{
    // Sim stuff
    Simulator editor;
    SimWorld sw;

    // Vis stuff
    VisWorld vw;
    VisLayer vl;

    // State
    SimObject selectedObject = null;
    SelectionAnimation animation = null;

    public BoltSim(VisWorld vw, VisLayer vl, VisConsole console, SimWorld sw)
    {
        editor = new Simulator(vw, vl, console, sw);
        this.sw = sw;

        this.vw = vw;
        this.vl = vl;

        // Customized event handling
        vl.addEventHandler(new BoltEventHandler());

        // Render updates about the world
        RenderThread rt = new RenderThread();
        rt.start();
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

            synchronized(sw) {
                for (SimObject obj : sw.objects) {

                    double d = Collisions.collisionDistance(ray.getSource(), ray.getDir(), obj.getShape(), obj.getPose());

                    if (d < bestd) {
                        selectedObject = obj;
                        bestd = d;
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

    public static void main(String args[])
    {
        GetOpt gopt = new GetOpt();
        gopt.addBoolean('h', "help", false, "Show this help");
        gopt.addString('w', "world", "", "World file");
        gopt.addString('c', "config", "", "Configuration file");
        gopt.addBoolean('\0', "start", false, "Start simulation automatically");
        gopt.addInt('\0', "fps", 10, "Maximum frame rate");

        if (!gopt.parse(args) || gopt.getBoolean("help") || gopt.getExtraArgs().size() > 0) {
            gopt.doHelp();
            return;
        }

        SimWorld world;
        try {
            Config config = new Config();
            if (gopt.wasSpecified("config"))
                config = new ConfigFile(EnvUtil.expandVariables(gopt.getString("config")));

            if (gopt.getString("world").length() > 0) {
                String worldFilePath = EnvUtil.expandVariables(gopt.getString("world"));
                world = new SimWorld(worldFilePath, config);
            } else {
                world = new SimWorld(config);
            }

        } catch (IOException ex) {
            System.out.println("ex: "+ex);
            ex.printStackTrace();
            return;
        }

        VisWorld vw = new VisWorld();
        VisLayer vl = new VisLayer(vw);
        VisCanvas vc = new VisCanvas(vl);
        vc.setTargetFPS(gopt.getInt("fps"));

        JFrame jf = new JFrame("Bolt Simulator");
        jf.setLayout(new BorderLayout());
        jf.add(vc, BorderLayout.CENTER);

        jf.setSize(800,600);
        jf.setVisible(true);
        jf.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);


        BoltSim boltsim = new BoltSim(vw, vl, new VisConsole(vw,vl,vc), world);

        if (gopt.getBoolean("start")) {
            world.setRunning(true);
        }

    }
}
