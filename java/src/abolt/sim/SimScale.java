package abolt.sim;

import java.util.ArrayList;

import abolt.bolt.BoltObject;
import abolt.bolt.ObjectEffector;
import abolt.classify.ClassifierManager;
import abolt.classify.Features;
import abolt.classify.Features.FeatureCategory;
import abolt.classify.GKNN;
import abolt.lcmtypes.category_t;
import april.sim.SimWorld;

public class SimScale extends SimLocation{

	public SimScale(SimWorld sw) {
		super(sw);
		
		Features.addFeaturePair(FeatureCategory.WEIGHT, category_t.CAT_WEIGHT);
		ClassifierManager.getSingleton().addClassifier(FeatureCategory.WEIGHT, new GKNN(10, 3));
		
		Features.addEffector(new ObjectEffector(){
			// If the object is over the scale, then add its weight as a feature
			public void effect(BoltObject obj) {
				double x = pose[0];
				double y = pose[1];
				double[] objPose = obj.getPos();
				if(objPose[0] >= x - size && objPose[0] <= x + size &&
						objPose[1] >= y - size && objPose[1] <= y + size){
					ArrayList<Double> weightFeatures = new ArrayList<Double>();
					double weight;
					if(obj.sourceObject != null){
						weight = obj.sourceObject.getFeatures(FeatureCategory.WEIGHT).get(0);
					} else {
						double[][] b = obj.getBBox();
						double vol2 = Math.pow(b[1][0] - b[0][0], 2) + Math.pow(b[1][1] - b[0][1], 2) + Math.pow(b[1][2] - b[0][2], 2);
						weight = Math.sqrt(vol2);
					}
					weightFeatures.add(weight);
					obj.addFeature(FeatureCategory.WEIGHT, weightFeatures);
				}
			}
		});
	}
}
