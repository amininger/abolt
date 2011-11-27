package abolt.sim;

import java.awt.Color;
import java.io.*;
import java.util.*;

import lcm.lcm.*;

import april.sim.*;
import april.jmat.*;
import april.vis.*;
import april.util.*;

public class SimChicken implements SimBoltObject, SimActionable, SimGrabbable
{
    double[][] pose;
    String name;
    HashMap<String, ArrayList<String> > actions = new HashMap<String, ArrayList<String> >();
    HashMap<String, String> currentState = new HashMap<String, String>();
    ArrayList<String> featureVec;
    int id;

    static final double extent = 0.025;
    static final double sensingRange = .5;
    static final double actionRange = .005;

    // Make Chicken model
    static VisObject visModel;
    static {
        VisChain vc = new VisChain(LinAlg.scale(extent),
                                   new VzCylinder(new VzMesh.Style(new Color(0xC2B280))));
        visModel = vc;
    }

    static Shape collisionShape;
    static {
        collisionShape = new SphereShape(-0.5*extent);
    }

    public SimChicken(SimWorld sw)
    {
        this(sw, "CHICKEN");
    }

    public SimChicken(SimWorld sw, String _name)
    {
        //pose = LinAlg.xytToMatrix(_xyt);
        name = _name;

        featureVec = new ArrayList<String>();
        // Temporary: populated with object color and dimensions and then randomness
        featureVec.add("TAN");
        featureVec.add("RAW");
        featureVec.add("DIRTY");
        featureVec.add("CYLINDER");

	// Add actions
        actions.put("CLEAN", new ArrayList<String>());
        actions.get("CLEAN").add("CLEAN");
        actions.get("CLEAN").add("DIRTY");
        currentState.put("CLEAN", "DIRTY");

        actions.put("COOKED", new ArrayList<String>());
        actions.get("COOKED").add("COOKED");
        actions.get("COOKED").add("RAW");
        currentState.put("COOKED", "RAW");

        actions.put("HELD", new ArrayList<String>());
        actions.get("HELD").add("TRUE");
        actions.get("HELD").add("FALSE");
        currentState.put("HELD", "FALSE");

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

    public void setLoc(double[] xyt){
	pose = LinAlg.xytToMatrix(xyt); // XXX : should maybe be a matrix transform
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

    public String[] getNounjectives()
    {
	String[] features = new String[featureVec.size()];
	for(int i=0; i<featureVec.size(); i++){
	    features[i] = featureVec.get(i);
	}

        return features;
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
