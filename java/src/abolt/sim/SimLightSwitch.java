package abolt.sim;

import java.awt.Color;
import java.io.*;

import lcm.lcm.*;

import april.sim.*;
import april.jmat.*;
import april.vis.*;
import april.util.*;

class SimLightSwitch implements SimObject
{
    double[][] pose;

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

    public SimLightSwitch(double[] _xyt)
    {
        pose = LinAlg.xytToMatrix(_xyt);
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
}
