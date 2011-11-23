package abolt.sim;

import java.awt.Color;
import java.io.*;
import java.util.*;

import lcm.lcm.*;

import april.sim.*;
import april.jmat.*;
import april.vis.*;
import april.util.*;

public class SimLightSwitch implements SimObject, SimSensable, SimActionable
{
    double[][] pose;
    String name;
    HashMap<String, ArrayList<String> > actions = new HashMap<String, ArrayList<String> >();
    HashMap<String, String> currentState = new HashMap<String, String>();
    ArrayList<String> featureVec;
    ArrayList<String> stateVec;
    int id;

    static final double baseExtent = 0.05;
    static final double switchRange = 0.1;
    static final double sensingRange = 0.3;

    // Make Dishwasher model
    static VisObject visModel;
    static {
        VisChain vc = new VisChain(LinAlg.translate(0,0,.001),
                                   LinAlg.scale(baseExtent, baseExtent, .001),
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
        this(sw, "LIGHT_SWITCH");
    }

    public SimLightSwitch(SimWorld sw, String _name)
    {
        //pose = LinAlg.xytToMatrix(_xyt);
        name = _name;

        featureVec = new ArrayList<String>();
        // Temporary: populated with object color and dimensions and then randomness
        featureVec.add("beige");
        featureVec.add("small");
        featureVec.add("rectangular");

        stateVec = new ArrayList<String>();
        stateVec.add("toggle = ON");

        // Add actions
        actions.put("TOGGLE", new ArrayList<String>());
        actions.get("TOGGLE").add("ON");
        actions.get("TOGGLE").add("OFF");
        currentState.put("TOGGLE", "OFF");

        Random r = new Random();
        id = r.nextInt();
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

    /*public String[] getNounjectives()
    {
        String[] nounjectives = new String[featureVec.size()];
        featureVec.toArray(nounjectives);
        return nounjectives;
    }*/
    public String getProperties()
    {
        return null;
    }

    public boolean inRange(double[] xyt)
    {
        double[] obj_xyt = LinAlg.matrixToXYT(pose);
        return LinAlg.distance(LinAlg.resize(obj_xyt, 2), LinAlg.resize(xyt, 2)) < sensingRange;
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

    // XXX Only lets you change one state at a time
    public void setState(String newState)
    {
        String[] keyValuePair = newState.split("=");
        currentState.put(keyValuePair[0], keyValuePair[1]);
    }
}
