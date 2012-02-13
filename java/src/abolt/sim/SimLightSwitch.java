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

public class SimLightSwitch implements SimObject, SimSensable, SimActionable
{
    double[][] pose;
    String name;
    HashMap<String, ArrayList<String> > actions = new HashMap<String, ArrayList<String> >();
    HashMap<String, String> currentState = new HashMap<String, String>();
    ArrayList<String> featureVec;
    int id;

    static final double baseExtent = 0.05;
    static final double switchRange = 0.1;
    static final double sensingRange = 0.3;
    static final double actionRange = 0.1;

    // Make Dishwasher model
    static VisObject visModel;
    static {
        VisChain vc = new VisChain(LinAlg.translate(0,0,.001),
                                   LinAlg.scale(baseExtent/2, baseExtent/2, .001),
                                   new VzBox(new VzMesh.Style(Color.black)),
                                   LinAlg.translate(0,0,.002),
                                   LinAlg.scale(.2, .2, 1),
                                   new VzBox(new VzMesh.Style(Color.yellow)));
        visModel = vc;
    }

    static Shape collisionShape;
    static {
        collisionShape = new SphereShape(-switchRange);
    }

    public SimLightSwitch(SimWorld sw)
    {
        name = "LIGHT_SWITCH";

        featureVec = new ArrayList<String>();
        featureVec.add("COLOR=BEIGE");
        featureVec.add("SIZE=SMALL");
        featureVec.add("SHAPE=RECTANGLE");
        featureVec.add("DEPTH=.02");

        // Add actions
        actions.put("TOGGLE", new ArrayList<String>());
        actions.get("TOGGLE").add("ON");
        actions.get("TOGGLE").add("OFF");
        currentState.put("TOGGLE", "OFF");

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
        if (currentState.get("TOGGLE").equals("OFF"))
            return visModel;
        else {
            VzRectangle sq = new VzRectangle(new VzMesh.Style(Color.yellow));
            VisChain vc = new VisChain(visModel,
                                       LinAlg.scale(1.3),
                                       LinAlg.translate(-0.4, 0.37),
                                       sq);
            return vc;
        }
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

    public String getName()
    {
        return name;
    }

    public int getID()
    {
        return id;
    }

    // Return comma-separated properties and pose at end(separated by space)
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
        String[] allkvpairs = newState.split(",");
        for(int i=0; i<allkvpairs.length; i++){
            String[] keyValuePair = newState.split("=");
            if ((actions.get(keyValuePair[0]) != null) &&
                actions.get(keyValuePair[0]).contains(keyValuePair[1]))
            {
                currentState.put(keyValuePair[0], keyValuePair[1]);
            }
        }
    }
}
