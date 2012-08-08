package abolt.sim;

import java.util.ArrayList;

import abolt.classify.ClassifierManager;
import abolt.classify.Features;
import abolt.classify.Features.FeatureCategory;
import abolt.classify.GKNN;
import abolt.lcmtypes.category_t;
import abolt.objects.BoltObject;
import abolt.objects.BoltObjectManager;
import abolt.objects.ObjectEffector;
import april.sim.SimWorld;

public class SimScale extends SimLocation{

	public SimScale(SimWorld sw) {
		super(sw);
		
		Features.addFeaturePair(FeatureCategory.WEIGHT, category_t.CAT_WEIGHT);
		ClassifierManager.getSingleton().addClassifier(FeatureCategory.WEIGHT, new GKNN(10, 3));
		
		BoltObjectManager.getSingleton().addEffector(new ObjectEffector(){
			// If the object is over the scale, then add its weight as a feature
			public void effect(BoltObject obj) {
				double x = pose[0];
				double y = pose[1];
				double[] objPose = obj.getPose();
				if(objPose[0] >= x - size && objPose[0] <= x + size &&
						objPose[1] >= y - size && objPose[1] <= y + size){
					if(obj.getInfo().createdFrom != null){
						double weight = obj.getInfo().createdFrom.getWeight();
						ArrayList<Double> weightFeatures = new ArrayList<Double>();
						weightFeatures.add(weight);
						obj.addFeatures(FeatureCategory.WEIGHT, weightFeatures);
					}
				}
			}
		});
		
	}
	

}
