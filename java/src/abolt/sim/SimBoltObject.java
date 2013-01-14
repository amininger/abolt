package abolt.sim;

import java.awt.Color;
import java.io.IOException;
import java.util.ArrayList;

import abolt.classify.Features.FeatureCategory;
import abolt.util.SimUtil;
import april.jmat.LinAlg;
import april.sim.SimObject;
import april.sim.Shape;
import april.util.StructureReader;
import april.util.StructureWriter;
import april.vis.VisObject;

public abstract class SimBoltObject implements SimObject
{	
	protected int id;
	protected double[] pose = new double[6];
	
	protected boolean isVisible = true;
	
	protected Shape shape = null;
	protected VisObject model = null;
	protected Color color = Color.black;
	
	public SimBoltObject(){
		id = SimUtil.nextID();
	}
	
	// Get/Set ID
	public int getID(){
		return id;
	}
	public void setID(int id){
		this.id = id;
	}
	
	// Get/Set Visible (hide/show the object)
	public boolean getVisible(){
		return isVisible;
	}
	public void setVisible(boolean isVisible){
    	this.isVisible = isVisible;
    }
	
	// Get the pose/Color/Shape of the object for rendering	
	public double[][] getPose(){
		return LinAlg.xyzrpyToMatrix(pose);
	}
	
	public double[] getPos(){
		return pose;
	}
	
	public void setPose(double[][] poseMatrix) {
		pose = LinAlg.matrixToXyzrpy(poseMatrix);
	}
	
	/**** Methods for visual appearance ****/
	public Color getColor(){
		return color;
	}
	
	public april.sim.Shape getShape() {
		return shape;
	}
	
	public abolt.collision.Shape getAboltShape(){
		return null;
	}
	
	public VisObject getVisObject() {
		return model;
	}
	
	// Return fetaures for a given category
	public ArrayList<Double> getFeatures(FeatureCategory fc){
		return null;
	}

	public void setRunning(boolean arg0) { }
	
	public abstract void read(StructureReader arg0) throws IOException;
	
	public abstract void write(StructureWriter arg0) throws IOException;
}

