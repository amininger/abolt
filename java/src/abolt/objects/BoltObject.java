package abolt.objects;

import java.awt.Color;
import java.awt.Rectangle;
import java.io.IOException;
import java.util.ArrayList;

import abolt.util.SimUtil;
import april.jmat.LinAlg;
import april.sim.BoxShape;
import april.sim.Shape;
import april.sim.SimObject;
import april.sim.SimWorld;
import april.sim.SphereShape;
import april.util.StructureReader;
import april.util.StructureWriter;
import april.util.TimeUtil;
import april.vis.VisChain;
import april.vis.VisObject;
import april.vis.VzBox;
import april.vis.VzMesh;

import abolt.classify.ColorFeatureExtractor;
import abolt.classify.ConfidenceLabel;
import abolt.classify.SizeFeatureExtractor;
import abolt.classify.Features.FeatureCategory;
import abolt.kinect.ObjectInfo;
import abolt.lcmtypes.object_data_t;

public class BoltObject{
    protected int id;
    
    protected double[] pos;
    protected double[][] bbox;

    protected LabelCollection labels;
    protected ObjectInfo info;
    
	protected Shape shape;
	protected VisChain model;
	protected VzBox vzBox;

    public BoltObject(SimWorld world)
    {
        this.id = SimUtil.nextID();
		this.bbox = new double[2][3];
		this.pos = new double[6];
        this.labels = new LabelCollection();
        this.info = null;
		shape = new SphereShape(.01);
        model = null;
    }
    
    public BoltObject(int id){
    	this.id = id;
    	this.labels = new LabelCollection();
		this.bbox = new double[2][3];
		this.pos = new double[6];
		this.info = null;
		shape = new SphereShape(.01);
        model = null;
    }
    
    public int getID(){
    	return id;
    }
   
    public double[] getPose(){
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
    
	public double[][] getPoseMatrix() {
		return LinAlg.xyzrpyToMatrix(pos);
	}
	
	public void setPose(double[][] arg0) {
		pos = LinAlg.matrixToXyzrpy(arg0);
	}
	
	public ObjectInfo getInfo(){
		return info;
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
	
	public ArrayList<Double> getFeatures(FeatureCategory cat) {
		return info.getFeatures(cat);
	}

	public Shape getShape() {
		return shape;
	}
	
	public VisObject getVisObject() {
		return model;
	}
	
	public void updateObject(ObjectInfo info){
		this.info = info;
		double[] bb = SizeFeatureExtractor.boundingBoxWorld(info.points);
        double[] min = new double[]{bb[0], bb[1], bb[2]};
        double[] max = new double[]{bb[3], bb[4], bb[5]};
        double[] xyzrpy = new double[]{0, 0, 0, 0, 0, 0};
        double maxDim = 0;
        for(int i = 0; i < 3; i++){
            xyzrpy[i] = (min[i] + max[i])/2;
            if(max[i] - min[i] > maxDim){
            	maxDim = max[i] - min[i];
            }
        }
        maxDim /= 2;
        double[] center = new double[]{xyzrpy[0], xyzrpy[1], xyzrpy[2]};

        LinAlg.minusEquals(min, center);
        LinAlg.minusEquals(max, center);
        
        shape = new SphereShape(maxDim);
        Color color = ColorFeatureExtractor.getColorFromFeatures(info.getFeatures(FeatureCategory.COLOR));
        model = new VisChain(LinAlg.translate(center), LinAlg.scale(max[0] - min[0], max[1] - min[1], max[2] - min[2]), 
                new VzBox(new VzMesh.Style(color)));
        bbox[0] = min;
        bbox[1] = max;
        pos = xyzrpy;
	}
}