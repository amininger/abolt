package abolt.sim;

import java.awt.*;
import java.awt.event.*;
import javax.swing.*;
import java.io.*;
import java.util.*;

import lcm.lcm.*;

import april.config.*;
import april.jmat.*;
import april.jmat.geom.*;
import april.sim.*;
import april.util.*;
import april.vis.*;

import abolt.vis.*;
import abolt.lcmtypes.*;

/** A variation of the core april-simulator with specific
 *  additions for BOLT. Gives the user access to the
 *  VisWorld to implement animations, etc. as desired as
 *  well as control the creation of simulated objects outside
 *  of the standard SimBox and SimBox. Just a wrapper around
 *  simulator! Not an extension, for now.
 */
public class BoltSim implements VisConsole.Listener
{
    static LCM lcm = LCM.getSingleton();

    // Sim stuff
    Simulator editor;
    SimWorld sw;

    // Vis stuff
    VisWorld vw;
    VisLayer vl;

    // State
    Object selectionLock = new Object();
    SimObject selectedObject = null;
    SelectionAnimation animation = null;

    // Tasks
    PeriodicTasks tasks = new PeriodicTasks(1);

    public BoltSim(VisWorld vw, VisLayer vl, VisConsole console, SimWorld sw)
    {
        console.addListener(this);
        editor = new Simulator(vw, vl, console, sw);
        this.sw = sw;

        this.vw = vw;
        this.vl = vl;


        // Customized event handling
        vl.addEventHandler(new BoltEventHandler());

        // Render updates about the world
        RenderThread rt = new RenderThread();
        rt.start();

        // Start periodic tasks
        tasks.addFixedDelay(new ObservationsTask(), 0.2);
    }

    public boolean consoleCommand(VisConsole console, PrintStream out, String command)
    {
        String toks[] = command.trim().split("\\s+");
        if (toks.length == 0)
            return false;

        // Respond to start/stop by manipulating observation task
        if (toks[0].equals("start")) {
            setRunning(true);
            return true;
        }

        if (toks[0].equals("stop")) {
            setRunning(false);
            return true;
        }

        return false;
    }

    public ArrayList<String> consoleCompletions(VisConsole console, String prefix)
    {
        return null;    // Only using start and stop from sim, still
    }

    public void setRunning(boolean run)
    {
        tasks.setRunning(run);
    }

    class ObservationsTask implements PeriodicTasks.Task
    {
        public void run(double dt)
        {
            observations_t obs = new observations_t();
            obs.utime = TimeUtil.utime();

            ArrayList<object_data_t> obsList = new ArrayList<object_data_t>();
            ArrayList<String> sensList = new ArrayList<String>();

            // Loop through the world and see if we sense anything. For now,
            // we "sense" something if we're within its sensing radius,
            // regardless of line-of-sight
            //
            // XXX Sensing is different now, too. Just assume we see everything
            synchronized (sw) {
                for (SimObject o: sw.objects) {
                    if (o instanceof SimSensable) {
                        SimSensable s = (SimSensable)o;

                        StringBuilder sb = new StringBuilder();
                        sb.append("ID="+s.getID()+",");
                        sb.append("NAME="+s.getName()+","); // For debugging convenience
                        sb.append(s.getProperties());
                        if (o instanceof SimActionable) {
                            SimActionable a = (SimActionable)o;
                            sb.append(a.getState());
                        }
                        sensList.add(sb.toString());
                    }
                    if (o instanceof SimBoltObject) {
                        SimBoltObject bo = (SimBoltObject)o;
                        object_data_t obj_data = new object_data_t();
                        obj_data.utime = TimeUtil.utime();
                        obj_data.id = bo.getID();
                        obj_data.nounjectives = bo.getNounjectives();   // XXX May change to doubles
                        obj_data.nj_len = obj_data.nounjectives.length;
                        obj_data.pos = LinAlg.matrixToXyzrpy(o.getPose());
                        obsList.add(obj_data);
                    }
                }
            }

            synchronized (selectionLock) {
                obs.click_id = -1;
                if (selectedObject != null) {
                    if (selectedObject instanceof SimSensable) {
                        obs.click_id = ((SimSensable)selectedObject).getID();
                    } else if (selectedObject instanceof SimBoltObject) {
                        obs.click_id = ((SimBoltObject)selectedObject).getID();
                    }
                }
            }
            obs.sensables = sensList.toArray(new String[0]);
            obs.nsens = obs.sensables.length;
            obs.observations = obsList.toArray(new object_data_t[0]);
            obs.nobs = obs.observations.length;

            lcm.publish("OBSERVATIONS",obs);
        }
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
            boltsim.setRunning(true);
        }

    }
}
