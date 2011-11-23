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
    ArrayList<String> featureVec;
    ArrayList<String> stateVec;
    int id;

    static final double baseExtent = 0.05;
    static final double switchRange = 0.1;

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

    public SimLightSwitch(SimWorld sw, String _name)
    {
        //pose = LinAlg.xytToMatrix(_xyt);
        name = _name;

        featureVec = new ArrayList<String>();
	featureVec.add("beige");
        featureVec.add("small");
        featureVec.add("rectangular");

	stateVec = new ArrayList<String>();
	stateVec.add("toggle = ON");

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

    public String[] getNounjectives()
    {
        String[] nounjectives = new String[featureVec.size()];
        featureVec.toArray(nounjectives);
	return nounjectives;
    }

    public String[] getAllowedStates()
    {
        String[] allStates = new String[stateVec.size()];
	stateVec.toArray(allStates);
        return allStates;
    }

    public String getState()
    {
        return stateVec.get(0); // XXX                                                                                                                                                                 
    }

    public void setState(String newState)
    {
        stateVec.set(0, newState); // XXX                                                                                                                                                              
    }
}
