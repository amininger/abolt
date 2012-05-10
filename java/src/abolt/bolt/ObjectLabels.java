package abolt.bolt;

import java.awt.Rectangle;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.Map;
import java.util.Queue;

import kinect.classify.ConfidenceLabel;
import kinect.classify.Features;
import kinect.classify.Features.FeatureCategory;
import kinect.kinect.ObjectInfo;
import kinect.lcmtypes.categorized_data_t;
import kinect.lcmtypes.category_t;

/**
 * @author Aaron
 * A container class designed to store the label history for an object
 * as well as the current best (highest confidence) labels
 */
public class ObjectLabels {
	private HashMap<FeatureCategory, Integer> queueSizes;
	private HashMap<FeatureCategory, Queue<ConfidenceLabel>> labels;
	private HashMap<FeatureCategory, ConfidenceLabel> bestLabels;
	
	public ObjectLabels(){
		labels = new HashMap<FeatureCategory, Queue<ConfidenceLabel> >();
		bestLabels = new HashMap<FeatureCategory, ConfidenceLabel>();
		for(FeatureCategory cat : FeatureCategory.values()){
			labels.put(cat, new LinkedList<ConfidenceLabel>());
			bestLabels.put(cat, new ConfidenceLabel(0, "unknown"));
		}
		
		queueSizes = new HashMap<FeatureCategory, Integer>();
		queueSizes.put(FeatureCategory.COLOR, 10);
		queueSizes.put(FeatureCategory.SHAPE, 15);
		queueSizes.put(FeatureCategory.SIZE, 10);
	}
	
	public categorized_data_t[] getCategorizedData(){
		categorized_data_t[] data = new categorized_data_t[labels.size()];
		int i = 0;
		for(Map.Entry<FeatureCategory, ConfidenceLabel> entry : bestLabels.entrySet()){
			categorized_data_t catDat = new categorized_data_t();
			catDat.cat = new category_t();
			catDat.cat.cat = Features.getLCMCategory(entry.getKey());
			catDat.len = 1;
			catDat.confidence = new double[1];
			catDat.label = new String[1];
			catDat.confidence[0] = entry.getValue().getConfidence();
			catDat.label[0] = entry.getValue().getLabel();
			data[i++] = catDat;
		}
		return data;
	}
	
	public ConfidenceLabel getBestLabel(FeatureCategory cat){
		return bestLabels.get(cat);
	}
	
	public void updateLabel(FeatureCategory cat, ConfidenceLabel newLabel){
		Queue<ConfidenceLabel> labelQueue = labels.get(cat);
		labelQueue.offer(newLabel);
		if(labelQueue.size() > queueSizes.get(cat)){
			labelQueue.remove();
		}

        double sum = 0;
        int count = 0;
        ArrayList<String> bestS = new ArrayList<String>();
        ArrayList<Integer> bestCount = new ArrayList<Integer>();
        int max = 0;
        int max2 = 0;
        int index;
        for (ConfidenceLabel c : labelQueue)
        {
            String label = c.getLabel();

            int cnt = 0;
            if ((index = bestS.indexOf(label)) >= 0)
            {
                cnt = bestCount.get(index) + 1;
                bestCount.set(index, cnt);
            }
            else
            {
                cnt = 1;
                bestS.add(label);
                bestCount.add(cnt);
            }
            if (cnt > max)
            {
                max = cnt;
            }
            sum+= c.getConfidence();
            count++;
        }
        
        double confidence = sum/(double)count * (double)max/(double)count;
        //best label
        if ((index = bestCount.indexOf(max)) >= 0)
        {
        	bestLabels.put(cat, new ConfidenceLabel(confidence, bestS.get(index)));
        }
	}
}
