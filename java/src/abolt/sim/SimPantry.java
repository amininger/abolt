package abolt.sim;

import java.awt.Color;
import java.io.*;
import java.util.*;

import lcm.lcm.*;

import april.sim.*;
import april.jmat.*;
import april.vis.*;
import april.util.*;

import abolt.util.*;

public class SimPantry implements SimObject, SimSensable, SimActionable
{
    // Simulator
    SimWorld sw;

    double[][] pose;
    String name;
    HashMap<String, ArrayList<String> > actions = new HashMap<String, ArrayList<String> >();
    HashMap<String, String> currentState = new HashMap<String, String>();
    ArrayList<String> featureVec;
    int id;

    static final double xextent = 0.2;
    static final double yextent = 0.4;
    static final double sensingRange = 0.5;
    static final double actionRange = 0.15;

    // Make Pantry model
    static VisObject visModel;
    static {
        VisChain vc = new VisChain(new VisChain(LinAlg.scale(xextent/2, yextent/2, 1),
                                                LinAlg.translate(0,0,0.001),
                                                new VzRectangle(new VzLines.Style(Color.green,2))),
                                   new VisChain(LinAlg.translate(-.8*xextent/2,0,0),
                                                LinAlg.rotateZ(-Math.PI/2),
                                                LinAlg.scale(.002),
                                                new VzText(VzText.ANCHOR.CENTER,
                                                           "<<green>> pantry")));
        visModel = vc;
    }

    static Shape collisionShape;
    static {
        collisionShape = new BoxShape(new double[] {xextent, yextent, 0});
    }

    public SimPantry(SimWorld _sw)
    {
        sw = _sw;

        name = "PANTRY";

        featureVec = new ArrayList<String>();
        featureVec.add("COLOR=GREEN");
        featureVec.add("SHAPE=CUBE");
        featureVec.add("SIZE=LARGE");
        featureVec.add("FULLNESS=STOCKED");

        // Add actions
        actions.put("DOOR", new ArrayList<String>());
        actions.get("DOOR").add("OPEN");
        actions.get("DOOR").add("OPEN");
        currentState.put("DOOR", "CLOSED");

        id = SimUtil.nextID();
    }

    public double[][] getPose()
    {
        return LinAlg.copy(pose);
    }

    public void setPose(double[][] T)
    {
        pose = LinAlg.copy(T);
    }

    public Shape getShape()
    {
        return collisionShape;
    }

    public VisObject getVisObject()
    {
        return visModel;
    }

    public void read(StructureReader ins) throws IOException
    {
        pose = LinAlg.xyzrpyToMatrix(ins.readDoubles());
    }

    public void write(StructureWriter outs) throws IOException
    {
        outs.writeComment("XYZRPY Truth");
        outs.writeDoubles(LinAlg.matrixToXyzrpy(pose));
    }

    public void setRunning(boolean run)
    {

    }

    public int getID()
    {
        return id;
    }

    public String getName()
    {
        return name;
    }

    public String getProperties()
    {
        StringBuilder properties = new StringBuilder();
        for(int i=0; i<featureVec.size(); i++){
            properties.append(featureVec.get(i)+",");
        }
        double[] xyzrpy = LinAlg.matrixToXyzrpy(pose);
        Formatter f = new Formatter();
        f.format("[%f %f %f %f %f %f]",
                 xyzrpy[0],
                 xyzrpy[1],
                 xyzrpy[2],
                 xyzrpy[3],
                 xyzrpy[4],
                 xyzrpy[5]);
        properties.append("POSE="+f.toString()+",");

        return properties.toString();
    }

    public boolean inSenseRange(double[] xyt)
    {
        double[] obj_xyt = LinAlg.matrixToXYT(pose);
        return LinAlg.distance(LinAlg.resize(obj_xyt, 2), LinAlg.resize(xyt, 2)) < sensingRange;
    }

    public boolean inActionRange(double[] xyt)
    {
        double[] obj_xyt = LinAlg.matrixToXYT(pose);
        return LinAlg.distance(LinAlg.resize(obj_xyt, 2), LinAlg.resize(xyt, 2)) < actionRange;
    }

    public String[] getAllowedStates()
    {
        ArrayList<String> allStates = new ArrayList<String>();
        for (String key: actions.keySet()) {
            for (String value: actions.get(key)) {
                allStates.add(key+"="+value);
            }
        }
        String[] stateArray = allStates.toArray(new String[0]);
        return stateArray;
    }

    public String getState()
    {
        StringBuilder state = new StringBuilder();
        for (String key: currentState.keySet()) {
            state.append(key+"="+currentState.get(key)+",");
        }
        return state.toString();
    }

    public void setState(String newState)
    {
        // XXX Handle grabbing...

        String[] allkvpairs = newState.split(",");
        for(int i=0; i<allkvpairs.length; i++){
            String[] keyValuePair = newState.split("=");
            if ((actions.get(keyValuePair[0]) != null) &&
                actions.get(keyValuePair[0]).contains(keyValuePair[1]))
            {
                currentState.put(keyValuePair[0], keyValuePair[1]);
            }
        }

        // Make chickens available if door is open
        /*if (currentState.get("DOOR").equals("OPEN") &&
            !currentState.containsKey("GRAB"))
        {
            currentState.put("GRAB","NOTHING");
            actions.put("GRAB",new ArrayList<String>());
            actions.get("GRAB").add("NOTHING");
            actions.get("GRAB").add("CHICKEN");
        } else if (currentState.get("DOOR").equals("CLOSED")) {
            currentState.remove("GRAB");
            actions.remove("GRAB");
        }*/
    }
}
