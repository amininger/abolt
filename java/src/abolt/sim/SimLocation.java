package abolt.sim;

import java.awt.Color;
import java.io.IOException;
import java.util.ArrayList;
import java.util.HashMap;

import april.jmat.LinAlg;
import april.sim.BoxShape;
import april.sim.Shape;
import april.sim.SimObject;
import april.sim.SimWorld;
import april.sim.SphereShape;
import april.util.StructureReader;
import april.util.StructureWriter;
import april.vis.VisChain;
import april.vis.VisObject;
import april.vis.VzLines;
import april.vis.VzMesh;
import april.vis.VzRectangle;
import april.vis.VzText;

import abolt.bolt.Bolt;
import abolt.classify.SimFeatures;
import abolt.classify.Features.FeatureCategory;
import abolt.objects.*;
import abolt.util.SimUtil;

public class SimLocation implements SimSensable, SimObject, SimActionable {
    protected VisObject model;
    protected Shape shape;

    protected int id;
    protected double[] pose;

    protected String name;
    protected Color color;

    protected double size = .1;
    protected SensableStates sensStates;

    public SimLocation(SimWorld sw)
    {
    	id = SimUtil.nextID();
    }

    public Shape getShape()
    {
        return shape;
    }

    public VisObject getVisObject()
    {
        return constructModel();
    }

	public String getName() {
		return name;
	}


	@Override
	public double[][] getPose() {
		return LinAlg.xyzrpyToMatrix(pose);
	}

	@Override
	public void setPose(double[][] poseMatrix) {
		pose = LinAlg.matrixToXyzrpy(poseMatrix);
	}

	@Override
	public void setRunning(boolean arg0) {
	}

	@Override
	public int getID() {
		return id;
	}

	public String getProperties() {
		String props = String.format("ID=%d,NAME=%s,", id, name);
		props += sensStates.getProperties();
		props += String.format("POSE=[%f %f %f %f %f %f],", pose[0], pose[1], pose[2], pose[3], pose[4], pose[5]);
		props += String.format("BBOX=[%f %f %f %f %f %f]", -size, -size, -size, size, size, size);

		return props;
	}

	public void read(StructureReader ins) throws IOException
    {
		double[] xy = ins.readDoubles();
    	pose = new double[]{xy[0], xy[1], 0, 0, 0, 0};

    	name = ins.readString();
    	
    	int[] colors = ins.readInts();
    	color = new Color(colors[0], colors[1], colors[2]);
    	
    	
        
        int numProps = ins.readInt();
        String[] props = new String[numProps];
        for(int i = 0; i < numProps; i++){
        	props[i] = ins.readString();
        }	
        sensStates = new SensableStates(props);

        shape = new BoxShape(new double[]{2*size, 2*size, 0});

        SensableManager.getSingleton().addSensable(this);
    }
	

    public void write(StructureWriter outs) throws IOException
    {
    	outs.writeComment("XY");
        outs.writeDoubles(new double[]{pose[0], pose[1]});
        outs.writeInts(new int[]{color.getRed(), color.getGreen(), color.getBlue()});
    }

	@Override
	public void setState(String keyValString) {
		sensStates.setState(keyValString);
	}
	
	@Override
	public boolean inSenseRange(double[] xyt) {
		return true;
	}
	
	private VisObject constructModel(){
		ArrayList<Object> objs = new ArrayList<Object>();

        // The larger box making up the background of the object
		objs.add(new VisChain(LinAlg.translate(0,0,.001),
				LinAlg.scale(size), new VzRectangle(new VzMesh.Style(color))));
		
		// The name of the location
		objs.add(new VisChain(LinAlg.rotateZ(Math.PI/2), LinAlg.translate(0,-.8*size,.007),
                LinAlg.scale(0.002),
                new VzText(VzText.ANCHOR.CENTER, String.format("<<black>> %s", name))));
	
		if(sensStates.getState("door") != null && sensStates.getState("door").equals("open")){
			// The smaller inner box is only drawn if it is open
			objs.add(new VisChain(LinAlg.translate(0,0,.004),
					LinAlg.scale(size*.9), new VzRectangle(new VzMesh.Style(Color.DARK_GRAY))));
		}
		
		return new VisChain(objs.toArray());
	}

}
