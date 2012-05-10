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

public class SimBlock implements SimObject, SimBoltObject, SimActionable, SimGrabbable
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


    private VisChain model;
    private Shape shape;
    private double size;

    private String sizeStr;
    private String shapeStr;
    private String colorStr;

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

    public SimBlock(SimWorld sw)
    {
        //pose = LinAlg.xytToMatrix(_xyt);
    	name = "BLOCK";

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

        // Holding actions
        actions.put("HELD", new ArrayList<String>());
        actions.get("HELD").add("TRUE");
        actions.get("HELD").add("FALSE");
        currentState.put("HELD", "FALSE");

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
        return shape;
    }

    public VisObject getVisObject()
    {
        return model;
    }

    public void read(StructureReader ins) throws IOException
    {
    	pose = LinAlg.xyzrpyToMatrix(ins.readDoubles());
        bbox = new double[][] {{-extent, -extent, -extent},
                               { extent,  extent,  extent}};

        colorStr = ins.readString();
        shapeStr = ins.readString();
        sizeStr = ins.readString();

        int colorInt = 0xA29260;
        if(colorStr.equals("RED")){
            colorInt = 0xFF0000;
        } else if(colorStr.equals("BLUE")){
            colorInt = 0x000088;
        } else if(colorStr.equals("GREEN")){
            colorInt = 0x00BB00;
        } else if(colorStr.equals("GRAY")){
            colorInt = 0x666666;
        }

        if(sizeStr.equals("LARGE")){
            this.size = 0.08;
        } else if(sizeStr.equals("MEDIUM")){
            this.size = 0.05;
        } else if(sizeStr.equals("SMALL")){
            this.size = 0.02;
        }

        if(shapeStr.equals("CYLINDER")){
            model = new VisChain(LinAlg.scale(this.size), LinAlg.translate(0,0,1),
                    new VzCylinder(new VzMesh.Style(new Color(colorInt))));
        } else if(shapeStr.equals("SPHERE")){
            model = new VisChain(LinAlg.scale(this.size), LinAlg.translate(0,0,1),
                    new VzSphere(new VzMesh.Style(new Color(colorInt))));
        } else {
        	//cube
            this.size *= 1.5;
            model = new VisChain(LinAlg.scale(this.size), LinAlg.translate(0,0,1),
                    new VzBox(new VzMesh.Style(new Color(colorInt))));
        }

        this.shape = new SphereShape(-this.size);

        categories = new HashMap<Integer, HashMap<String, Double> >();
        HashMap<String, Double> category;

        // COLOR
        category = new HashMap<String, Double>();
        category.put("color-" + colorStr, Math.random() * .3 + .7);
        categories.put(category_t.CAT_COLOR, category);

        // SHAPE
        category = new HashMap<String, Double>();
        category.put("shape-" + shapeStr, Math.random() * .3 + .7);
        categories.put(category_t.CAT_SHAPE, category);

        // SIZE
        category = new HashMap<String, Double>();
        category.put("size-" + sizeStr, Math.random() * .3 + .7);
        categories.put(category_t.CAT_SIZE, category);
    }

    public void write(StructureWriter outs) throws IOException
    {
    	outs.writeComment("XYZRPY Truth");
        outs.writeDoubles(LinAlg.matrixToXyzrpy(pose));
        outs.writeString(colorStr);
        outs.writeString(shapeStr);
        outs.writeString(sizeStr);
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

