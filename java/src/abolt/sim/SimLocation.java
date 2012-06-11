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
import april.vis.VzRectangle;
import april.vis.VzText;

import abolt.bolt.Bolt;
import abolt.classify.SimFeatures;
import abolt.classify.Features.FeatureCategory;
import abolt.objects.*;
import abolt.util.SimUtil;

public class SimLocation implements SimSensable, SimObject {
    protected VisObject model;
    protected Shape shape;

    protected int id;
    protected double[] pose;
    protected double[][] bbox;

    protected String name;
    protected String colorStr;

    protected double size = .12;
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
        return model;
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
		props += sensStates.getProperties() + ",";
		props += String.format("POSE=[%f %f %f %f %f %f],", pose[0], pose[1], pose[2], pose[3], pose[4], pose[5]);
		props += String.format("BBOX=[%f %f %f %f %f %f]", bbox[0][0], bbox[0][1], bbox[0][2], bbox[1][0], bbox[1][1], bbox[1][2]);

		return props;
	}

	public void read(StructureReader ins) throws IOException
    {
		double[] xy = ins.readDoubles();
    	pose = new double[]{xy[0], xy[1], size/2, 0, 0, 0};
    	bbox = new double[][]{new double[]{-size, -size, -size/2}, new double[]{size, size, size/2}};

    	name = ins.readString();
        colorStr = ins.readString();
        Color color = SimFeatures.getColorValue(colorStr);

        model =  new VisChain(new VisChain(LinAlg.translate(0,0,-size/2 + .001),
        						LinAlg.scale(size),
				                new VzRectangle(new VzLines.Style(color,2))),
				   new VisChain(LinAlg.rotateZ(Math.PI/2), LinAlg.translate(0,-.8*size/2,-size/2 + .001),
				                LinAlg.scale(0.002),
				                new VzText(VzText.ANCHOR.CENTER, String.format("<<%s>> %s", colorStr, name))));
        
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
        outs.writeString(colorStr);
    }

	@Override
	public boolean inSenseRange(double[] xyt) {
		return true;
	}

}
