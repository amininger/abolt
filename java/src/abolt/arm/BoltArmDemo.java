package abolt.arm;

import java.awt.*;
import java.awt.event.*;
import javax.swing.*;
import java.util.*;
import java.io.*;

import lcm.lcm.*;

import april.dynamixel.*;
import april.jserial.*;
import april.jmat.*;
import april.jmat.geom.*;
import april.vis.*;
import april.util.*;

import abolt.lcmtypes.*;

public class BoltArmDemo implements LCMSubscriber
{
    // LCM
    LCM lcm = LCM.getSingleton();

    // Arm joints
    ArrayList<Joint> joints;

    // Rendering thread
    RenderThread rt;

    // Simulation thread
    SimulationThread st;

    ExpiringMessageCache<dynamixel_status_list_t> statuses = new ExpiringMessageCache<dynamixel_status_list_t>(0.2, true);
    ExpiringMessageCache<dynamixel_command_list_t> cmds = new ExpiringMessageCache<dynamixel_command_list_t>(0.2, true);

    // Command line flags/options
    GetOpt opts;

    public BoltArmDemo(GetOpt opts_)
    {
        opts = opts_;
        joints = BoltArm.initArm();

        // We're going to spoof these if simming, so don't send them
        if (!opts.getBoolean("sim")) {
            lcm.subscribe("ARM_STATUS", this);
        } else {
            st = new SimulationThread();
            st.start();
        }
        lcm.subscribe("ARM_COMMAND", this);

        rt = new RenderThread();
        rt.start();
    }

    public void messageReceived(LCM lcm, String channel, LCMDataInputStream ins)
    {
        try {
            messageReceivedEx(lcm, channel, ins);
        } catch (IOException ex) {
            System.out.println("ERR: "+ex);
        }
    }

    public void messageReceivedEx(LCM lcm, String channel, LCMDataInputStream ins) throws IOException
    {
        if (channel.equals("ARM_STATUS")) {
            // If non-simulated, render the arm position from THESE
            dynamixel_status_list_t status = new dynamixel_status_list_t(ins);
            long utime = Long.MAX_VALUE;
            for (dynamixel_status_t s: status.statuses) {
                utime = Math.min(utime, s.utime);
            }
            statuses.put(status, utime);
        } else if (channel.equals("ARM_COMMAND")) {
            // If simulated, use these to render the arm position
            dynamixel_command_list_t cmdl = new dynamixel_command_list_t(ins);
            long utime = Long.MAX_VALUE;
            for (dynamixel_command_t c: cmdl.commands) {
                utime = Math.min(utime, c.utime);
            }
            cmds.put(cmdl, utime);
        }
    }

    static enum ActionState
    {
        POINT, GRAB, DROP
    }

    class RenderThread extends Thread
    {
        int fps = 60;

        VisWorld vw;
        VisLayer vl;
        VisCanvas vc;

        double[] xyz = null;

        public RenderThread()
        {
            JFrame jf = new JFrame("Bolt Arm Simulation Demo");
            jf.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
            jf.setLayout(new BorderLayout());
            jf.setSize(1920, 1080);

            vw = new VisWorld();
            vl = new VisLayer(vw);
            vc = new VisCanvas(vl);
            jf.add(vc, BorderLayout.CENTER);

            // Grid
            VzGrid.addGrid(vw);

            // Default zoom
            vl.cameraManager.fit2D(new double[] {-1,-1}, new double[] {1,1}, true);

            // Event handler
            vl.addEventHandler(new GoalEventHandler());

            jf.setVisible(true);
        }

        public void run()
        {
            while (true) {
                // Render Arm
                {
                    dynamixel_status_list_t dsl = statuses.get();
                    dynamixel_command_list_t dcl = cmds.get();
                    if (opts.getBoolean("sim") && dcl != null) {
                        for (int i = 0; i < dcl.len; i++) {
                            joints.get(i).set(dcl.commands[i].position_radians);
                        }
                    } else if (dsl != null) {
                        for (int i = 0; i < dsl.len; i++) {
                            joints.get(i).set(dsl.statuses[i].position_radians);
                        }
                    }

                    VisWorld.Buffer vb = vw.getBuffer("arm");
                    vb.addBack(new VisChain(LinAlg.rotateZ(-Math.PI/2),
                                            new VzTriangle(0.08, 0.08, 0.08,
                                                           new VzMesh.Style(Color.green))));
                    vb.addBack(new VisChain(LinAlg.translate(0,0,BoltArm.baseHeight/2),
                                            new VzBox(0.04, 0.04, BoltArm.baseHeight,
                                                      new VzMesh.Style(Color.black))));

                    double[][] xform = LinAlg.translate(0,0,BoltArm.baseHeight);
                    for (Joint j: joints) {
                        LinAlg.timesEquals(xform, j.getRotation());
                        vb.addBack(new VisChain(LinAlg.copy(xform),
                                                j.getVis()));
                        LinAlg.timesEquals(xform, j.getTranslation());
                    }
                    vb.swap();
                }

                // Draw current goal
                {
                    VisWorld.Buffer vb = vw.getBuffer("goal");
                    if (xyz != null) {
                        vb.addBack(new VisChain(LinAlg.translate(xyz),
                                                new VzCircle(0.015, new VzMesh.Style(Color.yellow))));
                    }
                    vb.swap();
                }

                TimeUtil.sleep(1000/fps);
            }
        }

        /** Set the arm goal point */
        public void setGoal(double[] goal)
        {
            xyz = LinAlg.copy(goal);
        }
    }

    class GoalEventHandler extends VisEventAdapter
    {
        public int getDispatchOrder()
        {
            return 0;
        }

        robot_command_t getRobotCommand(double[] dest, ActionState state)
        {
            robot_command_t cmd = new robot_command_t();
            cmd.utime = TimeUtil.utime();
            cmd.updateDest = (dest != null);
            cmd.dest = LinAlg.resize(dest, 6);
            switch (state) {
                case POINT:
                    cmd.action = "POINT";
                    break;
                case GRAB:
                    cmd.action = "GRAB";
                    break;
                case DROP:
                    cmd.action = "DROP";
                    break;
                default:
                    cmd.action = "POINT";
                    break;
            }

            return cmd;
        }

        public boolean mousePressed(VisCanvas vc, VisLayer vl, VisCanvas.RenderInfo rinfo, GRay3D ray, MouseEvent e)
        {
            double[] xyz = ray.intersectPlaneXY();
            int mods = e.getModifiersEx();
            boolean shift = (mods & MouseEvent.SHIFT_DOWN_MASK) > 0;
            boolean ctrl = (mods & MouseEvent.CTRL_DOWN_MASK) > 0;
            if (shift && !ctrl) {
                lcm.publish("ROBOT_COMMAND", getRobotCommand(xyz, ActionState.POINT));
                rt.setGoal(xyz);
                return true;
            } else if (!shift && ctrl) {
                lcm.publish("ROBOT_COMMAND", getRobotCommand(xyz, ActionState.DROP));
                rt.setGoal(xyz);
                return true;
            } else if (shift && ctrl) {
                lcm.publish("ROBOT_COMMAND", getRobotCommand(xyz, ActionState.GRAB));
                rt.setGoal(xyz);
                return true;
            }

            return false;
        }

        public boolean mouseDragged(VisCanvas vc, VisLayer vl, VisCanvas.RenderInfo rinfo, GRay3D ray, MouseEvent e)
        {
            double[] xyz = ray.intersectPlaneXY();
            int mods = e.getModifiersEx();
            boolean shift = (mods & MouseEvent.SHIFT_DOWN_MASK) > 0;
            boolean ctrl = (mods & MouseEvent.CTRL_DOWN_MASK) > 0;
            if (shift && !ctrl) {
                lcm.publish("ROBOT_COMMAND", getRobotCommand(xyz, ActionState.POINT));
                rt.setGoal(xyz);
                return true;
            } else if (!shift && ctrl) {
                lcm.publish("ROBOT_COMMAND", getRobotCommand(xyz, ActionState.DROP));
                rt.setGoal(xyz);
                return true;
            } else if (shift && ctrl) {
                lcm.publish("ROBOT_COMMAND", getRobotCommand(xyz, ActionState.GRAB));
                rt.setGoal(xyz);
                return true;
            }

            return false;
        }
    }

    class SimulationThread extends Thread
    {
        int Hz = 100;

        public void run()
        {
            while (true) {
                TimeUtil.sleep(1000/Hz);

                long utime = TimeUtil.utime();
                dynamixel_status_list_t dsl = new dynamixel_status_list_t();
                dsl.len = joints.size();
                dsl.statuses = new dynamixel_status_t[joints.size()];
                for (int i = 0; i < joints.size(); i++) {
                    dynamixel_status_t status = new dynamixel_status_t();
                    status.utime = utime;

                    Joint j = joints.get(i);
                    if (j instanceof RevoluteJoint) {
                        RevoluteJoint rj = (RevoluteJoint)j;
                        status.position_radians = rj.getAngle();
                    } else if (j instanceof HandJoint) {
                        HandJoint hj = (HandJoint)j;
                        status.position_radians = hj.getAngle();
                    }

                    dsl.statuses[i] = status;
                }

                lcm.publish("ARM_STATUS", dsl);
            }
        }
    }

    static public void main(String[] args)
    {
        GetOpt opts = new GetOpt();
        opts.addBoolean('s',"sim",false,"Run in simulation mode");
        opts.addBoolean('h',"help",false,"Display this help screen");
        opts.addBoolean('l',"lcm",false,"Emit LCM");

        if (!opts.parse(args)) {
            System.err.println("ERR: Option error - "+opts.getReason());
            return;
        }

        if (opts.getBoolean("help")) {
            opts.doHelp();
            return;
        }

        BoltArmDemo bd = new BoltArmDemo(opts);
    }
}
