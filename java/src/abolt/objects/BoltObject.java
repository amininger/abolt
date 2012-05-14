package abolt.objects;

import java.awt.Color;
import java.awt.Rectangle;
import java.io.IOException;
import java.util.ArrayList;

import abolt.util.SimUtil;
import april.jmat.LinAlg;
import april.sim.Shape;
import april.sim.SimObject;
import april.sim.SimWorld;
import april.util.StructureReader;
import april.util.StructureWriter;
import april.util.TimeUtil;
import april.vis.VisObject;

import abolt.classify.Features.FeatureCategory;
import abolt.kinect.ObjectInfo;
import abolt.lcmtypes.object_data_t;

public abstract class BoltObject implements SimObject{
    protected int id;
    
    protected double[] pos;
    protected double[][] bbox;

    protected LabelCollection labels;

    public BoltObject(SimWorld world)
    {
        this.id = SimUtil.nextID();
		this.bbox = new double[2][3];
		this.pos = new double[6];
        this.labels = new LabelCollection();
    }
    
    public BoltObject(int id){
    	this.id = id;
    	this.labels = new LabelCollection();
		this.bbox = new double[2][3];
		this.pos = new double[6];
    }
    
    public int getID(){
    	return id;
    }
   
    public double[] getPos(){
    	return pos;
    }
    
    public void setPos(double[] pos){
    	this.pos = pos;
    }
    
    public double[][] getBBox(){
    	return bbox;
    }
    
    public LabelCollection getLabels(){
    	return labels;
    }
    
	public double[][] getPose() {
		return LinAlg.xyzrpyToMatrix(pos);
	}
	
	public void setPose(double[][] arg0) {
		pos = LinAlg.matrixToXyzrpy(arg0);
	}
	
	public object_data_t getData(){
		object_data_t data = new object_data_t();
		data.utime = TimeUtil.utime();
		data.id = id;
		data.pos = pos;
		data.bbox = bbox;
		data.cat_dat = labels.getCategorizedData();
		data.num_cat = data.cat_dat.length;
		return data;
	}
	
	public abstract ArrayList<Double> getFeatures(FeatureCategory cat);

	public abstract Shape getShape();

	public abstract VisObject getVisObject();

	public void setRunning(boolean running) { }

	public void write(StructureWriter sw) throws IOException { }
	
	public void read(StructureReader sr) throws IOException { }
	
	
	
}
