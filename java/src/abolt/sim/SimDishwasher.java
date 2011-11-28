package abolt.sim;

import java.awt.Color;
import java.io.*;
import java.util.*;

import lcm.lcm.*;

import april.sim.*;
import april.jmat.*;
import april.vis.*;
import april.util.*;

public class SimDishwasher implements SimObject, SimSensable, SimActionable
{
    double[][] pose;
    String name;
    ArrayList<String> featureVec;
    HashMap<String, ArrayList<String> > actions = new HashMap<String, ArrayList<String> >();
    HashMap<String, String> currentState = new HashMap<String, String>();
    int id;

    static final double extent = 0.2;
    static final double sensingRange = 0.5;
    static final double actionRange = 0.15;

    // Make Dishwasher model
    static VisObject visModel;
    static {
        VisChain vc = new VisChain(LinAlg.scale(extent),
                                   new VzSquare(new VzLines.Style(Color.cyan,2)),
                                   LinAlg.translate(0,-2*extent,0),
                                   LinAlg.scale(.005),
                                   new VzText(VzText.ANCHOR.CENTER,
                                              "<<cyan>> dishwasher"));
        visModel = vc;
    }

    static Shape collisionShape;
    static {
        collisionShape = new SphereShape(-0.5*extent);
    }

    public SimDishwasher(SimWorld sw)
    {
        this(sw, "DISHWASHER");
    }

    public SimDishwasher(SimWorld sw, String _name)
    {
        //pose = LinAlg.xytToMatrix(_xyt);
        name = _name;

        featureVec = new ArrayList<String>();
        featureVec.add("COLOR=CYAN");
        featureVec.add("SHAPE=CUBE");
        featureVec.add("FULL=FALSE");
        featureVec.add("SHAPE=CUBE");

        // Add actions
        actions.put("TOGGLE", new ArrayList<String>());
        actions.get("TOGGLE").add("ON");
        actions.get("TOGGLE").add("OFF");
        currentState.put("TOGGLE", "OFF");

        actions.put("DOOR", new ArrayList<String>());
        actions.get("DOOR").add("OPEN");
        actions.get("DOOR").add("CLOSED");
        currentState.put("DOOR", "CLOSED");

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

    public String getProperties()
    {
        StringBuilder properties = new StringBuilder();
        for(int i=0; i<featureVec.size(); i++){
            properties.append(featureVec.get(i)+",");
        }
        double[] xyt = LinAlg.matrixToXYT(pose);
        properties.append("["+xyt[0]+" "+xyt[1]+" "+xyt[2]+"],"); //XXX format better

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
            if(actions.get(keyValuePair[0]).contains(keyValuePair[1])){
                currentState.put(keyValuePair[0], keyValuePair[1]);
            }
        }
    }
}
