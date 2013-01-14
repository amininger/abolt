package abolt.bolt;

import java.awt.Color;
import java.awt.image.BufferedImage;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;

import april.jmat.LinAlg;
import april.sim.Shape;
import april.sim.SphereShape;
import april.vis.VisChain;
import april.vis.VisObject;
import april.vis.VzBox;
import april.vis.VzMesh;

import abolt.classify.*;
import abolt.classify.Features.FeatureCategory;
import abolt.kinect.ObjectInfo;
import abolt.sim.SensableStates;
import abolt.sim.SimBoltObject;
import abolt.sim.SimLocation;

public class BoltObject{
    protected int id;

    protected double[] pos;
    protected double[][] bbox;
    protected double radius;
    
    protected ArrayList<double[]> points;
	
	protected BufferedImage projection;
	
	protected HashMap<String, String> objectState;
	
	public SimBoltObject sourceObject = null;
	
    private HashMap<FeatureCategory, ArrayList<Double> > features;
    
	public BoltObject(int id, ArrayList<double[]> points, BufferedImage projection){
		this.id = id;
		this.points = points;
		this.projection = projection;
		
		// Calculate bounding box and position
		double[] bb = SizeFeatureExtractor.boundingBox(points);
		bbox = new double[2][3];
        double[] min = new double[]{bb[0], bb[1], bb[2]};
        double[] max = new double[]{bb[3], bb[4], bb[5]};
        pos = LinAlg.scale(LinAlg.add(min, max), .5);	// (min + max)/2
        pos = new double[]{pos[0], pos[1], pos[2], 0, 0, 0};
        
        bbox[0] = LinAlg.subtract(min, pos);
        bbox[1] = LinAlg.subtract(max, pos);
        
        computeRadius();
        
        objectState = new HashMap<String, String>();
        features = new HashMap<FeatureCategory, ArrayList<Double> >();
	}
    
    public BoltObject(SimLocation loc){
    	sourceObject = loc;
    	this.id = loc.getID();
    	this.points = null;
    	this.projection = null;
    	this.pos = loc.getPos();
    	this.bbox = loc.getBBox();
        computeRadius();
    	this.objectState = loc.getStates().getCurrentState();
        features = new HashMap<FeatureCategory, ArrayList<Double> >();
    }
    
    private void computeRadius(){
        // Calculate the radius
        radius = 0;
        for(int i = 0; i < 2; i++){
        	for(int j = 0; j < 3; j++){
        		if(Math.abs(bbox[i][j]) > radius){
        			radius = Math.abs(bbox[i][j]);
        		}
        	}
        }
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
    
    public ArrayList<double[]> getPoints(){
    	return points;
    }

	public double getRadius(){
		return radius;
	}
	
	public BufferedImage getProjection(){
		return projection;
	}
	
	public ArrayList<Double> getFeatures(FeatureCategory cat) {
		return features.get(cat);
	}
	
	public void addFeature(FeatureCategory cat, ArrayList<Double> features){
		this.features.put(cat, features);
	}
	
	public boolean isVisible(){
		if(sourceObject == null){
			return true;
		} else {
			return sourceObject.getVisible();
		}
	}
	
	public String[] getLabels(){
		String[] labels = new String[objectState.size()];
		int i = 0;
		for(Map.Entry<String, String> e : objectState.entrySet()){
			labels[i++] = e.getKey() + "=" + e.getValue();
		}
		return labels;
	}
}
