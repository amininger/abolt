package abolt.sim;

import java.awt.Color;
import java.io.*;
import java.util.*;

import lcm.lcm.*;

import april.sim.*;
import april.jmat.*;
import april.vis.*;
import april.util.*;

import abolt.lcmtypes.categorized_data_t;
import abolt.lcmtypes.category_t;
import abolt.util.*;

public class SimChicken implements SimObject, SimBoltObject, SimActionable, SimGrabbable
{
    double[][] pose;
    double[][] bbox;
    String name;
    HashMap<String, ArrayList<String> > actions = new HashMap<String, ArrayList<String> >();
    HashMap<String, String> currentState = new HashMap<String, String>();
    HashMap<Integer, HashMap<String, Double> > categories;
    int id;

    static final double extent = 0.03;
    static final double sensingRange = .5;
    static final double actionRange = .1;

    // Make Chicken model
    static VisObject visModel;
    static {
        VisChain vc = new VisChain(LinAlg.scale(extent),
                                   LinAlg.translate(0,0,1),
                                   new VzCylinder(new VzMesh.Style(new Color(0xA29260))));
        visModel = vc;
    }

    static Shape collisionShape;
    static {
        // XXX Really should be collidable, but for now, not
        collisionShape = new SphereShape(-extent);
    }

    public SimChicken(SimWorld sw)
    {
        //pose = LinAlg.xytToMatrix(_xyt);
        name = "CHICKEN";
        
        bbox = new double[3][2];
        for(int i = 0; i < 3; i++){
        	bbox[i][0] = -extent;
        	bbox[i][1] = extent;
        }

        categories = new HashMap<Integer, HashMap<String, Double> >();
        HashMap<String, Double> category;
        
        // COLOR
        category = new HashMap<String, Double>();
        category.put("TAN", .9);
        categories.put(category_t.CAT_COLOR, category);
        
        // SHAPE
        category = new HashMap<String, Double>();
        category.put("CYLINDER", .92);
        categories.put(category_t.CAT_SHAPE, category);
        
        // SIZE
        category = new HashMap<String, Double>();
        category.put("MEDIUM", .87);
        categories.put(category_t.CAT_SIZE, category);

        // Add actions
        actions.put("CLEAN", new ArrayList<String>());
        actions.get("CLEAN").add("CLEAN");
        actions.get("CLEAN").add("DIRTY");
        currentState.put("CLEAN", "DIRTY");

        actions.put("COOKED", new ArrayList<String>());
        actions.get("COOKED").add("COOKED");
        actions.get("COOKED").add("RAW");
        currentState.put("COOKED", "RAW");  // NOTE: Sim does not reflect the one-way transition from raw to cooked. You cannot uncook something.

        //Random r = new Random();
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
    
    public double[][] getBBox()
    {
    	return LinAlg.copy(bbox);
    }

    public void setLoc(double[] xyzrpy){
        pose = LinAlg.xyzrpyToMatrix(xyzrpy);
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
    
    public categorized_data_t[] getCategorizedData(){
    	categorized_data_t[] data = new categorized_data_t[categories.size()];
    	int i = 0;
    	for(Map.Entry<Integer, HashMap<String, Double> > category : categories.entrySet()){
    		// For each category, create a new categorized_data_t and populate
    		data[i] = new categorized_data_t();
    		data[i].cat = new category_t();
    		data[i].cat.cat = category.getKey();
    		// Each <String, Double> pair in catLabels is a label and associated confidence
    		HashMap<String, Double> catLabels = category.getValue();
    		data[i].len = catLabels.size();
    		data[i].label = new String[catLabels.size()];
    		data[i].confidence = new double[catLabels.size()];
    		int j = 0;
    		for(Map.Entry<String, Double> label : catLabels.entrySet()){
    			data[i].label[j] = label.getKey();
    			data[i].confidence[j] = label.getValue();
    			j++;
    		}
    		i++;
    	}
    	
    	return data;
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
            if((actions.get(keyValuePair[0]) != null) &&
               actions.get(keyValuePair[0]).contains(keyValuePair[1]))
            {
                currentState.put(keyValuePair[0], keyValuePair[1]);
            }
        }
    }


}
