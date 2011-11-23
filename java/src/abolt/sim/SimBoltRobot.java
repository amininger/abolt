package abolt.sim;

import java.util.*;
import java.awt.Color;
import java.io.*;

import lcm.lcm.*;

import april.jmat.*;
import april.vis.*;
import april.lcm.*;
import april.util.*;
import april.sim.*;
import april.lcmtypes.*;

import abolt.lcmtypes.*;

public class SimBoltRobot implements SimObject, LCMSubscriber
{
    static LCM lcm = LCM.getSingleton();

    static VisObject visModel;
    static {
        VisChain vc = new VisChain(LinAlg.scale(.01), // Now units are in centimeters
                                   LinAlg.translate(0,0,2),
                                   new VzBox(10,10,4, new VzMesh.Style(Color.black)),
                                   LinAlg.translate(1.5,0,3),
                                   new VzBox(6,4,2, new VzMesh.Style(Color.red)),
                                   LinAlg.translate(-5,0,-.99),
                                   LinAlg.rotateZ(Math.PI/2),
                                   LinAlg.scale(1.5/12),
                                   new VzText(VzText.ANCHOR.CENTER,
                                              "<<monospaced-12,white >>bolt-robot"));
        visModel = vc;
    }

    PeriodicTasks tasks = new PeriodicTasks(2);

    ExpiringMessageCache<gamepad_t> gamepadCache = new ExpiringMessageCache<gamepad_t>(0.25);
    robot_command_t cmds; // most recent command received from somewhere
    DifferentialDrive drive;


    // The following are hacks! XXX
    static double kitchen[][] = {{1,-.2},{1.5,.3}};
    static double stove[][] = {{1,-.2}, {1.25,.05}};

    static double kitchenBox[][] = {{kitchen[0][0],kitchen[0][1], 0},
                                    {kitchen[0][0],kitchen[1][1], 0},
                                    {kitchen[1][0],kitchen[1][1], 0},
                                    {kitchen[1][0],kitchen[0][1], 0}};

    static double stoveBox[][] = {{stove[0][0],stove[0][1]},
                                  {stove[0][0],stove[1][1]},
                                  {stove[1][0],stove[1][1]},
                                  {stove[1][0],stove[0][1]}};


    static boolean kitchenLight = false;

    public SimBoltRobot(SimWorld sw)
    {
        drive = new DifferentialDrive(sw, this, new double[3]);
        drive.baseline = .1;
        drive.wheelDiameter = .04;

        lcm.subscribe("ROBOT_COMMAND", this);
        lcm.subscribe("GAMEPAD", this);


        tasks.addFixedDelay(new ObservationsTask(), .2);
        tasks.addFixedDelay(new ControlTask(), .04);


    }

    /** Where is the object? (4x4 matrix). It is safe to return your internal representation. **/
    public double[][] getPose()
    {
        return LinAlg.quatPosToMatrix(drive.poseTruth.orientation,
                                      drive.poseTruth.pos);
    }

    public void setPose(double T[][])
    {
        drive.poseTruth.orientation = LinAlg.matrixToQuat(T);
        drive.poseTruth.pos = new double[] { T[0][3], T[1][3], 0 };
    }

    /** What is the shape of this object? (Relative to the origin)**/
    public Shape getShape()
    {
        return new SphereShape(.05); // Robot has radius of 5 centimeters
    }

    /** What does the object LOOK like? (should be drawn at the
     * origin).
     **/
    public VisObject getVisObject()
    {
        return visModel;
    }

    /** Restore state that was previously written **/
    public void read(StructureReader ins) throws IOException
    {
        double Ttruth[][] = LinAlg.xyzrpyToMatrix(ins.readDoubles());
        double Todom[][] = LinAlg.xyzrpyToMatrix(ins.readDoubles());

        synchronized(drive) {
            drive.poseTruth.orientation = LinAlg.matrixToQuat(Ttruth);
            drive.poseTruth.pos = new double[] { Ttruth[0][3], Ttruth[1][3], Ttruth[2][3] };

            drive.poseOdom.orientation = LinAlg.matrixToQuat(Todom);
            drive.poseOdom.pos = new double[] { Todom[0][3], Todom[1][3], Todom[2][3] };
        }

    }

    /** Write one or more lines that serialize this instance. No line
     * is allowed to consist of just an asterisk. **/
    public void write(StructureWriter outs) throws IOException
    {
        double Ttruth[][] = null;
        double Todom[][] = null;
        synchronized(drive) {
            Ttruth = LinAlg.quatPosToMatrix(drive.poseTruth.orientation, drive.poseTruth.pos);
            Todom = LinAlg.quatPosToMatrix(drive.poseOdom.orientation, drive.poseOdom.pos);
        }
        outs.writeComment("XYZRPY Truth");
        outs.writeDoubles(LinAlg.matrixToXyzrpy(Ttruth));
        outs.writeComment("XYZRPY Odom");
        outs.writeDoubles(LinAlg.matrixToXyzrpy(Todom));

    }

    /** If the object does "fancy" stuff (spawning threads to simulate
     * a robot, for example), this method should be supported to allow
     * cleanly starting/stopping that other stuff. Objects should
     * begin in the NON running state. **/
    public void setRunning(boolean run)
    {
        drive.setRunning(run);
        tasks.setRunning(run);
    }


    boolean released = true; //XXX

    public void messageReceived(LCM lcm, String channel, LCMDataInputStream  dins)
    {
        try {
            if (channel.equals("GAMEPAD")) {
                gamepad_t msg = new gamepad_t(dins);
                gamepadCache.put(msg, msg.utime);

                if ((msg.buttons & 2048) == 2048) { // left stick button
                    if (released) {

                        robot_command_t toggle_light = new robot_command_t();
                        toggle_light.dest = new double[3];
                        if (kitchenLight)
                            toggle_light.action =  "NAME=SWITCH,STATE=OFF";
                        else
                            toggle_light.action =  "NAME=SWITCH,STATE=ON";

                        lcm.publish("ROBOT_COMMAND", toggle_light);
                        released = false;
                   }
                } else {
                    released = true;
                }

            } else if (channel.equals("ROBOT_COMMAND")) {
                cmds = new robot_command_t(dins);
                if (cmds.action != null)
                    processAction(cmds.action);

            }
        } catch(IOException e) {

        }

    }

    public void processAction(String action)
    {
        double pos[] = drive.poseTruth.pos;

        if (pos[0] >  kitchen[0][0] && pos[0] < kitchen[1][0] &&
            pos[1] > kitchen[0][1] && pos[1] < kitchen[1][1]) {

            if (action.startsWith("NAME=SWITCH,STATE=")) {
                String act = action.substring(action.lastIndexOf("=") + 1);
                if (act.equals("ON")) {
                    kitchenLight = true;
                } else if (act.equals("OFF"))  {
                    kitchenLight = false;
                }
            }
        }
    }


    class ObservationsTask implements PeriodicTasks.Task
    {
        public void run(double dt)
        {

            observations_t obs = new observations_t();
            obs.utime = TimeUtil.utime();


            double pos[] = drive.poseTruth.pos;
            double rpy[] = LinAlg.quatToRollPitchYaw(drive.poseTruth.orientation);

            ArrayList<object_data_t> obsList = new ArrayList<object_data_t>();
            ArrayList<String> sensList = new ArrayList<String>();


            // Always include the robot position
            sensList.add(String.format("ROBOT_POS=[%.3f %.3f %.4f],BATTERY=%.1f",
                                       pos[0],pos[1],rpy[2], 11.1));


            // Stove
            if (pos[0] >  stove[0][0] && pos[0] < stove[1][0] &&
                pos[1] > stove[0][1] && pos[1] < stove[1][1]) {
                object_data_t stove_data = new object_data_t();
                stove_data.utime = TimeUtil.utime();
                stove_data.id = 17;
                stove_data.nounjectives = new String[]{"HOTNESS=375","COLOR=white"};
                stove_data.nj_len = stove_data.nounjectives.length;

                stove_data.pos = LinAlg.resize(LinAlg.scale(LinAlg.add(stove[0],stove[1]),.5),3);
                stove_data.pos[2] = 14*Math.PI; // No heading information
                obsList.add(stove_data);
            }

            // Kitchen
            if (pos[0] >  kitchen[0][0] && pos[0] < kitchen[1][0] &&
                pos[1] > kitchen[0][1] && pos[1] < kitchen[1][1]) {
                Calendar cal = Calendar.getInstance();
                sensList.add(String.format("NAME=CLOCK,STATE=%dh%dm",cal.get(Calendar.HOUR_OF_DAY), cal.get(Calendar.MINUTE)));
                sensList.add("NAME=SWITCH,STATE="+(kitchenLight? "ON" : "OFF"));
            }

            obs.sensibles = sensList.toArray(new String[0]);
            obs.nsens = obs.sensibles.length;
            obs.observations = obsList.toArray(new object_data_t[0]);
            obs.nobs = obs.observations.length;




            lcm.publish("OBSERVATIONS",obs);
        }
    }

    class ControlTask implements PeriodicTasks.Task
    {
        public void run(double dt)
        {
            double mcmd[] = new double[2];

            gamepad_t gp = gamepadCache.get();
            if (gp != null) {

                final int RIGHT_VERT_AXIS = 3;
                final int RIGHT_HORZ_AXIS = 2;

                double speed = -gp.axes[RIGHT_VERT_AXIS];
                if ((gp.buttons&(16|64)) != 0)
                    speed *= 4;

                double turn = gp.axes[RIGHT_HORZ_AXIS];

                if (gp.buttons != 0) {
                    mcmd = new double[] { speed + turn, speed - turn };
                }
            } else {
                System.out.println("GAMEPAD NULL");
            }

            drive.motorCommands = mcmd;
        }
    }


}
