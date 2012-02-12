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
import abolt.util.*;

public class SimBoltRobot implements SimObject, LCMSubscriber
{
    static LCM lcm = LCM.getSingleton();

    // XXX Out of date. Eventually make this an arm
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

    // Robot positional information. XXX Not an arm, yet. We
    // just simulate having a magical floating end effector
    double[][] effectorPose;
    double armRange = 5.0 * .3048; // ft -> meters. Much larger than reality right now

    // Object we're holding onto
    SimGrabbable grabbedObject = null;

    // Object we're "pointing" to (designated in some fashion)
    SimObject pointTarget = null;

    SimWorld sw;

    public SimBoltRobot(SimWorld sw)
    {
        this.sw = sw;

        lcm.subscribe("ROBOT_COMMAND", this);
        lcm.subscribe("GAMEPAD", this);

        tasks.addFixedDelay(new ObservationsTask(), .2);
    }

    /** Where is the object? (4x4 matrix). It is safe to return your internal representation. **/
    public double[][] getPose()
    {
        return effectorPose;
    }

    public void setPose(double T[][])
    {
        effectorPose = T;
        // Update held object
        if (grabbedObject != null) {
            grabbedObject.setLoc(LinAlg.matrixToXyzrpy(T));
        }
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
        effectorPose = LinAlg.xyzrpyToMatrix(ins.readDoubles());
    }

    /** Write one or more lines that serialize this instance. No line
     * is allowed to consist of just an asterisk. **/
    public void write(StructureWriter outs) throws IOException
    {
        outs.writeComment("XYZRPY effector");
        outs.writeDoubles(LinAlg.matrixToXyzrpy(effectorPose));
    }

    /** If the object does "fancy" stuff (spawning threads to simulate
     * a robot, for example), this method should be supported to allow
     * cleanly starting/stopping that other stuff. Objects should
     * begin in the NON running state. **/
    public void setRunning(boolean run)
    {
        tasks.setRunning(run);
    }

    boolean released2 = true;
    boolean released3 = true;
    boolean released4 = true;

    public void messageReceived(LCM lcm, String channel, LCMDataInputStream  dins)
    {
        try {
            if (channel.equals("GAMEPAD")) {
                gamepad_t msg = new gamepad_t(dins);
                gamepadCache.put(msg, msg.utime);

                if ((msg.buttons & 2) == 2) { // #2 button toggles all actions in range (crazy, but whatever)
                    if (released2) {
                        ArrayList<robot_command_t> actions = new ArrayList<robot_command_t>();

                        for(SimObject o: sw.objects){
                            if (o instanceof SimActionable) {
                                SimActionable a = (SimActionable)o;
                                String[] possStates = a.getAllowedStates();
                                String[] states = a.getState().split(",");
                                String name = null;
                                int id = -1;
                                if (o instanceof SimSensable) {
                                    SimSensable s = (SimSensable)o;
                                    id = s.getID();
                                }
                                else if (o instanceof SimBoltObject) {
                                    SimBoltObject bo = (SimBoltObject)o;
                                    id = bo.getID();
                                }
                                for (String state: states) {
                                    String[] keyValuePair = state.split("=");
                                    ArrayList<String> values = SimUtil.getPossibleValues(possStates,
                                                                                         keyValuePair[0]);
                                    robot_command_t action = new robot_command_t();
                                    action.utime = TimeUtil.utime();
                                    action.updateDest = false;
                                    action.dest = LinAlg.matrixToXyzrpy(effectorPose);

                                    action.action = (name == null ? ("ID="+id) : ("NAME="+name))+","+keyValuePair[0]+"="+SimUtil.nextValue(values, keyValuePair[1]);
                                    actions.add(action);
                                }
                            }
                        }
                        for (robot_command_t action: actions) {
                            lcm.publish("ROBOT_COMMAND", action);
                        }

                        released2 = false;
                    }
                } else {
                    released2 = true;
                }

                // XXX Some more action spoofing for testing
                if ((msg.buttons & 8) == 8) {
                    robot_command_t action = new robot_command_t();
                    action.utime = TimeUtil.utime();
                    action.updateDest = true;
                    action.dest = new double[] {1.5, .5, 0, 0, 0, 0};
                    action.action = "";

                    lcm.publish("ROBOT_COMMAND", action);
                }

                // XXX Some more action spoofing for testing
                if ((msg.buttons & 4) == 4) {
                    robot_command_t action = new robot_command_t();
                    action.utime = TimeUtil.utime();
                    action.updateDest = true;
                    action.dest = new double[] {2.1, .2, 0, 0, 0, 0};
                    action.action = "";

                    lcm.publish("ROBOT_COMMAND", action);
                }

            } else if (channel.equals("ROBOT_COMMAND")) {
                cmds = new robot_command_t(dins);

                // Update position
                if (cmds.updateDest)
                    setPose(LinAlg.xyzrpyToMatrix(cmds.dest));

                // Execute action(s)
                if (cmds.action.length() > 0)
                    processAction(cmds.action);

            }
        } catch(IOException e) {

        }

    }

    /** Process a single action of the form NAME=(OBJ_NAME),(ACTION)=(STATE).
     *  If interacting with a "real" object, use ID=(#),(ACTION)=(STATE)
     */
    public void processAction(String action)
    {
        // Split up tokens
        String[] pairs = action.split(",");
        assert(pairs.length > 1);

        double[] xyzbot = LinAlg.resize(LinAlg.matrixToXyzrpy(getPose()), 3);

        synchronized(sw) {
            for (SimObject o: sw.objects) {
                if (o instanceof SimActionable) {
                    SimActionable a = (SimActionable)o;

                    // If not within action range, do not change state. This
                    // means that it is outside the play range of the arm,
                    // for now. XXX In reality, we should have to move the arm
                    // close enough to grab/interact, but for now, MAGIC!
                    double[][] T = o.getPose();
                    double[] xyzobj = LinAlg.resize(LinAlg.matrixToXyzrpy(T), 3);
                    if (LinAlg.distance(xyzbot, xyzobj) > armRange) {
                        continue;
                    }

                    // Check for holding action. If we ask to grab something
                    // and we aren't holding anything, pick it up. (This also
                    // teleports the gripper to this object)
                    String grab = SimUtil.getTokenValue(pairs[1], "HELD");
                    if (grab != null &&
                        grab.equals("TRUE") &&
                        grabbedObject == null &&
                        (o instanceof SimGrabbable))
                    {
                        grabbedObject = (SimGrabbable)o;
                        setPose(o.getPose());   // Teleport to object
                    }
                    else if (grab != null &&
                             grab.equals("FALSE") &&
                             grabbedObject != null)
                    {
                        if (grabbedObject == o) {
                            grabbedObject = null;
                        }
                    }

                    // Update the state of the object in question
                    if (pairs[0].startsWith("NAME=")) {
                        if (o instanceof SimSensable) {
                            SimSensable s = (SimSensable)o;
                            String name = s.getName();
                            if (!pairs[0].equals("NAME="+name))
                                continue;
                            a.setState(pairs[1]);
                        }
                    } else if (pairs[0].startsWith("ID=")) {
                        if (o instanceof SimBoltObject) {
                            SimBoltObject bo = (SimBoltObject)o;
                            int ID = bo.getID();
                            String[] tokens = pairs[0].split("=");
                            assert(tokens.length > 1);
                            if (Integer.valueOf(tokens[1]) != ID)
                                continue;
                            a.setState(pairs[1]);
                        } else if (o instanceof SimSensable) {
                            SimSensable s = (SimSensable)o;
                            int ID = s.getID();
                            String[] tokens = pairs[0].split("=");
                            assert(tokens.length > 1);
                            if (Integer.valueOf(tokens[1]) != ID)
                                continue;
                            a.setState(pairs[1]);
                        }
                    }
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

            //double pos[] = drive.poseTruth.pos;
            //double rpy[] = LinAlg.quatToRollPitchYaw(drive.poseTruth.orientation);
            double[] xyzrpy = LinAlg.matrixToXyzrpy(effectorPose);

            ArrayList<object_data_t> obsList = new ArrayList<object_data_t>();
            ArrayList<String> sensList = new ArrayList<String>();


            // Always include the robot position
            sensList.add(String.format("NAME=ROBOT,ROBOT_POS=[%.3f %.3f %.3f %.4f %.4f %.4f],BATTERY=%.1f",
                                       xyzrpy[0], xyzrpy[1], xyzrpy[2],
                                       xyzrpy[3], xyzrpy[4], xyzrpy[5],
                                       11.1));


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
                        sb.append("NAME="+s.getName()+",");
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
                        obsList.add(obj_data);
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
}
