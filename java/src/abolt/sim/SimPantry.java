package abolt.sim;

import java.awt.Color;
import java.io.*;

import lcm.lcm.*;

import april.sim.*;
import april.jmat.*;
import april.vis.*;
import april.util.*;

class SimPantry implements SimObject
{
    double[][] pose;

    static final double xextent = 0.2;
    static final double yextent = 0.4;

    // Make Dishwasher model
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
        collisionShape = new SphereShape(-Math.max(yextent,xextent));
    }

    public SimPantry(double[] _xyt)
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
