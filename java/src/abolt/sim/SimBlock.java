package abolt.sim;

import java.awt.Color;
import java.io.*;
import java.util.*;


import april.sim.*;
import april.util.*;

import abolt.classify.SimFeatures;
import abolt.classify.Features.FeatureCategory;

public class SimBlock extends SimBoltObject
{
    private String sizeStr;
    private String shapeStr;
    private String colorStr;
    private HashMap<FeatureCategory, ArrayList<Double>> features;

    private abolt.collision.Shape aboltShape;

    public SimBlock(SimWorld sw)
    {
    	features = new HashMap<FeatureCategory, ArrayList<Double>>();
    }
    
    public abolt.collision.Shape getAboltShape(){
    	return aboltShape;
    }
    
	@Override
	public ArrayList<Double> getFeatures(FeatureCategory fc) {
		return features.get(fc);
	}
	
    public void read(StructureReader ins) throws IOException
    {
    	pose = ins.readDoubles();
        colorStr = ins.readString();
        shapeStr = ins.readString();
        sizeStr = ins.readString();
        
        ArrayList<Double> weightFeatures = new ArrayList<Double>();
        weightFeatures.add(ins.readDouble());
        features.put(FeatureCategory.WEIGHT, weightFeatures);
        
        ArrayList<Double> squishinessFeatures = new ArrayList<Double>();
        squishinessFeatures.add(ins.readDouble());
        features.put(FeatureCategory.SQUISHINESS, squishinessFeatures);
        
        isVisible = (ins.readInt() == 1);
        color = SimFeatures.getColorValue(colorStr);
        double sizeScale = .05 * SimFeatures.getSizeValue(sizeStr);
        aboltShape = SimFeatures.getShape(shapeStr, sizeScale);
        shape = new SphereShape(sizeScale);
    }

    public void write(StructureWriter outs) throws IOException
    {
    	outs.writeComment("XYZRPY Truth");
        outs.writeDoubles(pose);
        outs.writeString(colorStr);
        outs.writeString(shapeStr);
        outs.writeString(sizeStr);
        outs.writeDouble(features.get(FeatureCategory.WEIGHT).get(0));
        outs.writeDouble(features.get(FeatureCategory.SQUISHINESS).get(0));
        outs.writeInt(isVisible ? 1 : 0);
    }

}

