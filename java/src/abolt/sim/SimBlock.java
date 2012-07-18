package abolt.sim;

import java.awt.Color;
import java.io.*;
import java.util.*;

import lcm.lcm.*;

import april.sim.*;
import april.jmat.*;
import april.vis.*;
import april.util.*;

import abolt.bolt.Bolt;
import abolt.classify.SimFeatures;
import abolt.classify.Features.FeatureCategory;
import abolt.lcmtypes.categorized_data_t;
import abolt.lcmtypes.category_t;
import abolt.objects.BoltObject;
import abolt.objects.ISimBoltObject;
import abolt.util.*;

public class SimBlock implements SimGrabbable, ISimBoltObject
{
    private String sizeStr;
    private String shapeStr;
    private String colorStr;
    private double weight;
    private Color color;
    protected double[][] pose;
    protected int id;
    protected Shape shape;
    protected VisObject model;

    private abolt.collision.Shape aboltShape;

    public SimBlock(SimWorld sw)
    {
    	id = SimUtil.nextID();
    }

	@Override
	public int getID() {
		return id;
	}
	
	public void setID(int id){
		this.id = id;
	}


    @Override
    public Color getColor(){
    	return color;
    }
    
    public double getWeight(){
    	return weight;
    }

    @Override
	public boolean inActionRange(double[] xyt) {
		return true;
	}

	@Override
	public Shape getShape() {
		return shape;
	}

	@Override
	public VisObject getVisObject() {
		return null;
	}

	@Override
	public double[][] getPose() {
		return pose;
	}

	@Override
	public void setPose(double[][] pose) {
		this.pose = pose;
	}

	@Override
	public void setPos(double[] xyzrpy) {
		this.pose = LinAlg.xyzrpyToMatrix(xyzrpy);
	}

	@Override
	public void setRunning(boolean arg0) {
	}

    @Override
    public abolt.collision.Shape getAboltShape(){
    	return aboltShape;
    }

    public void read(StructureReader ins) throws IOException
    {
    	pose = LinAlg.xyzrpyToMatrix(ins.readDoubles());
        colorStr = ins.readString();
        shapeStr = ins.readString();
        sizeStr = ins.readString();
        weight = ins.readDouble();
        color = SimFeatures.getColorValue(colorStr);
        double sizeScale = .05 * SimFeatures.getSizeValue(sizeStr);
        aboltShape = SimFeatures.getShape(shapeStr, sizeScale);
        shape = new SphereShape(sizeScale);
    }

    public void write(StructureWriter outs) throws IOException
    {
    	outs.writeComment("XYZRPY Truth");
        outs.writeDoubles(LinAlg.matrixToXyzrpy(pose));
        outs.writeString(colorStr);
        outs.writeString(shapeStr);
        outs.writeString(sizeStr);
        outs.writeDouble(weight);
    }

}

