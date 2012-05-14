package abolt.sim;

import java.awt.Color;
import java.io.IOException;
import java.util.ArrayList;
import java.util.HashMap;

import abolt.bolt.Bolt;
import abolt.classify.ClassifierManager;
import abolt.classify.SimFeatures;
import abolt.classify.Features.FeatureCategory;
import abolt.objects.BoltObject;
import april.jmat.LinAlg;
import april.sim.BoxShape;
import april.sim.Shape;
import april.sim.SimWorld;
import april.sim.SphereShape;
import april.util.StructureReader;
import april.util.StructureWriter;
import april.vis.VisChain;
import april.vis.VisObject;
import april.vis.VzLines;
import april.vis.VzRectangle;
import april.vis.VzText;

public class SimLocation extends BoltObject implements SimSensable {
    protected VisObject model;
    protected Shape shape;
    
    private String name;
    private String colorStr;
    
    protected double size = .2;

    public SimLocation(SimWorld sw)
    {
    	super(sw);
    }
    
    public Shape getShape()
    {
        return shape;
    }

    public VisObject getVisObject()
    {
        return model;
    }
    
	@Override
	public ArrayList<Double> getFeatures(FeatureCategory cat) {
		return null;
	}
	
	public String getName() {
		return name;
	}

	public String getProperties() {
		String props = String.format("ID=%d,NAME=%s,", id, name);
		props += String.format("POSE=[%f %f %f %f %f %f],", pos[0], pos[1], pos[2], pos[3], pos[4], pos[5]);
		props += String.format("BBOX=[%f %f %f %f %f %f]", bbox[0][0], bbox[0][1], bbox[0][2], bbox[1][0], bbox[1][1], bbox[1][2]);
		return props;
	}
	
	public void read(StructureReader ins) throws IOException
    {
		double[] xy = ins.readDoubles();
    	pos = new double[]{xy[0], xy[1], size/2, 0, 0, 0};
    	bbox = new double[][]{new double[]{-size, -size, -size/2}, new double[]{size, size, size/2}};

    	name = ins.readString();
        colorStr = ins.readString();
        Color color = SimFeatures.getColorValue(colorStr);
        
        model =  new VisChain(new VisChain(LinAlg.translate(0,0,-size/2 + .001),
        						LinAlg.scale(size),
				                new VzRectangle(new VzLines.Style(color,2))),
				   new VisChain(LinAlg.translate(0,-.8*size/2,-size/2 + .001),
				                LinAlg.scale(0.002),
				                new VzText(VzText.ANCHOR.CENTER, String.format("<<%s>> %s", colorStr, name))));

        shape = new BoxShape(new double[]{2*size, 2*size, 0});
        
        
    	if(Bolt.getSensableManager() != null){
            Bolt.getSensableManager().addSensable(this);
    	}
    }

    public void write(StructureWriter outs) throws IOException
    {
    	outs.writeComment("XY");
        outs.writeDoubles(new double[]{pos[0], pos[1]});
        outs.writeString(colorStr);
    }

	@Override
	public boolean inSenseRange(double[] xyt) {
		return true;
	}
}
