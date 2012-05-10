package abolt.bolt;

import java.util.ArrayList;

import abolt.classify.Features;
import abolt.classify.Features.FeatureCategory;
import abolt.classify.SizeFeatureExtractor;
import abolt.kinect.ObjectInfo;
import april.jmat.LinAlg;
import april.sim.Shape;
import april.vis.VisObject;

public class WorldBoltObject extends BoltObject {
	private ObjectInfo info;
	

	public WorldBoltObject(int id) {
		super(id);
	}

	@Override
	public ArrayList<Double> getFeatures(FeatureCategory cat) {
		// TODO Auto-generated method stub
		return null;
	}

	@Override
	public Shape getShape() {
		// TODO Auto-generated method stub
		return null;
	}

	@Override
	public VisObject getVisObject() {
		// TODO Auto-generated method stub
		return null;
	}
	
	public void updateObject(ObjectInfo info){
		double[] bb = SizeFeatureExtractor.boundingBoxWorld(info.points);
        double[] min = new double[]{bb[0], bb[1], bb[2]};
        double[] max = new double[]{bb[3], bb[4], bb[5]};
        double[] xyzrpy = new double[]{0, 0, 0, 0, 0, 0};
        for(int i = 0; i < 3; i++){
            xyzrpy[i] = (min[i] + max[i])/2;
        }
        double[] center = new double[]{xyzrpy[0], xyzrpy[1], xyzrpy[2]};

        LinAlg.minusEquals(min, center);
        LinAlg.minusEquals(max, center);
        
        bbox[0] = min;
        bbox[1] = max;
        pos = xyzrpy;
	}

}
