package abolt.objects;

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
import abolt.lcmtypes.object_data_t;
import abolt.sim.SimGrabbable;
import abolt.util.*;

public abstract class SimBoltObject extends BoltObject implements SimGrabbable
{
    protected double sensingRange;
    protected double actionRange;

    protected VisObject model;
    protected Shape shape;
    
    protected HashMap<FeatureCategory, ArrayList<Double>> features;

    public SimBoltObject(SimWorld sw)
    {
    	super(sw);
        features = new HashMap<FeatureCategory, ArrayList<Double>>();
        sensingRange = .5;
        actionRange = .1;
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
		return features.get(cat);
	}
	
	@Override
	public object_data_t getData(){
		if(inSensingRange()){
			return super.getData();
		}
		return null;
	}
	
    public boolean inActionRange(double[] xyt)
    {
        double[] obj_xyt = LinAlg.matrixToXYT(getPose());
        return LinAlg.distance(LinAlg.resize(obj_xyt, 2), LinAlg.resize(xyt, 2)) < actionRange;
    }
    
    public boolean inSensingRange()
    {
    	if(pos[0] < -.5 || pos[0] > .5 || pos[1] < -.5 || pos[1] > .5){
    		return false;
    	}
    	return true;
    }
}

