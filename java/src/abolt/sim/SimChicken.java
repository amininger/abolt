package abolt.sim;

import java.awt.Color;
import java.io.*;
import java.util.*;

import lcm.lcm.*;

import april.sim.*;
import april.jmat.*;
import april.vis.*;
import april.util.*;

public class SimChicken implements SimObject, SimSensable
{
    double[][] pose;
    String name;
    ArrayList<String> featureVec;
    int id;

    static final double extent = 0.025;

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

    public SimChicken(SimWorld sw, String _name)
    {
        //pose = LinAlg.xytToMatrix(_xyt);
        name = _name;

        featureVec = new ArrayList<String>();
        // Temporary: populated with object color and dimensions and then randomness                                                                                                                   
        featureVec.add("brown");
        featureVec.add("raw");
	featureVec.add("dirty");

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
}
