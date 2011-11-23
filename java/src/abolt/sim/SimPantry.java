package abolt.sim;

import java.awt.Color;
import java.io.*;
import java.util.*;

import lcm.lcm.*;

import april.sim.*;
import april.jmat.*;
import april.vis.*;
import april.util.*;

public class SimPantry implements SimBoltObject, SimSensable, SimActionable
{
    double[][] pose;
    String name;
    ArrayList<String> featureVec;
    ArrayList<String> stateVec;
    int id;

    static final double xextent = 0.2;
    static final double yextent = 0.4;
    static final double sensingRange = 0.5;

    // Make Pantry model
    static VisObject visModel;
    static {
        VisChain vc = new VisChain(LinAlg.scale(xextent, yextent, 1),
                                   new VzSquare(new VzLines.Style(Color.green,2)),
                                   LinAlg.translate(-2*xextent,0,0),
                                   LinAlg.rotateZ(-Math.PI/2),
                                   LinAlg.scale(.005),
                                   new VzText(VzText.ANCHOR.CENTER,
                                              "<<green>> pantry"));
        visModel = vc;
    }

    static Shape collisionShape;
    static {
        collisionShape = new SphereShape(-0.5*Math.max(yextent,xextent));
    }

    public SimPantry(SimWorld sw)
    {
        this(sw, "PANTRY");
    }

    public SimPantry(SimWorld sw, String _name)
    {
        name = _name;
        //pose = LinAlg.xytToMatrix(_xyt);

        featureVec = new ArrayList<String>();
        featureVec.add("green");
        featureVec.add("stocked");

        stateVec = new ArrayList<String>();
        stateVec.add("door = CLOSED");

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

    public boolean inRange(double[] xyt)
    {
        double[] obj_xyt = LinAlg.matrixToXYT(pose);
        return LinAlg.distance(LinAlg.resize(obj_xyt, 2), LinAlg.resize(xyt, 2)) < sensingRange;
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
