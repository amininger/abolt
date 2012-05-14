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
import abolt.classify.ClassifierManager;
import abolt.classify.SimFeatures;
import abolt.classify.Features.FeatureCategory;
import abolt.lcmtypes.categorized_data_t;
import abolt.lcmtypes.category_t;
import abolt.objects.BoltObject;
import abolt.objects.SimBoltObject;
import abolt.util.*;

public class SimBlock extends SimBoltObject implements SimGrabbable
{
    private String sizeStr;
    private String shapeStr;
    private String colorStr;

    public SimBlock(SimWorld sw)
    {
    	super(sw);
    }
    

    public void read(StructureReader ins) throws IOException
    {
    	pos = ins.readDoubles();

        colorStr = ins.readString();
        shapeStr = ins.readString();
        sizeStr = ins.readString();
        
        features.put(FeatureCategory.COLOR, SimFeatures.getColorFeatures(colorStr));
        features.put(FeatureCategory.SHAPE, SimFeatures.getShapeFeatures(shapeStr));
        features.put(FeatureCategory.SIZE, SimFeatures.getSizeFeatures(sizeStr));
        
        ClassifierManager cm = Bolt.getClassifierManager();
        for(FeatureCategory cat : FeatureCategory.values()){
        	if(features.get(cat) != null){
        		labels.updateLabel(cat, cm.classify(cat, this));
        	}
        }
        
        Color color = SimFeatures.getColorValue(colorStr);
        double sizeScale = .05 * SimFeatures.getSizeValue(sizeStr);
        model = SimFeatures.constructVisObject(shapeStr, color, sizeScale);
        shape = new SphereShape(sizeScale);
        
    	bbox = new double[][]{new double[]{-sizeScale, -sizeScale, -sizeScale}, new double[]{sizeScale, sizeScale, sizeScale}};
        
    	if(Bolt.getObjectManager() != null){
            Bolt.getObjectManager().addObject(this);
    	}
    }

    public void write(StructureWriter outs) throws IOException
    {
    	outs.writeComment("XYZRPY Truth");
        outs.writeDoubles(LinAlg.matrixToXyzrpy(getPose()));
        outs.writeString(colorStr);
        outs.writeString(shapeStr);
        outs.writeString(sizeStr);
    }
}

