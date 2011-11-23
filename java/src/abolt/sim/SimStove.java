package abolt.sim;

import java.awt.Color;
import java.io.*;

import lcm.lcm.*;

import april.sim.*;
import april.jmat.*;
import april.vis.*;
import april.util.*;

public class SimStove implements SimObject
{
    double[][] pose;

    static final double extent = 0.2;

    // Make stove model
    static VisObject visModel;
    static {
        VisChain vc = new VisChain(LinAlg.scale(extent),
                                   new VzSquare(new VzLines.Style(Color.red,2)),
                                   LinAlg.translate(0,-2*extent,0),
                                   LinAlg.scale(.005),
                                   new VzText(VzText.ANCHOR.CENTER,
                                              "<<red>> stove"));

        visModel = vc;
    }

    static Shape collisionShape;
    static {
        collisionShape = new SphereShape(-0.5*extent);
    }

    public SimStove(SimWorld sw)
    {
        //pose = LinAlg.xytToMatrix(_xyt);
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
