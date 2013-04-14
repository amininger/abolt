package abolt.bolt;

import java.io.IOException;
import java.util.ArrayList;

import abolt.classify.ClassifierManager;
import abolt.classify.Features;
import abolt.classify.Features.FeatureCategory;
import abolt.lcmtypes.training_data_t;
import abolt.lcmtypes.training_label_t;
import lcm.lcm.LCM;
import lcm.lcm.LCMDataInputStream;
import lcm.lcm.LCMSubscriber;

public class LCMReceiver implements LCMSubscriber {
	static LCMReceiver singleton = null;
	public static LCMReceiver getSingleton(){
		return singleton;
	}
	
	public static void Initialize(){
		singleton = new LCMReceiver();
	}
	
	private static LCM lcm = LCM.getSingleton();
	private String acks = "";
	
	public LCMReceiver(){
        lcm.subscribe("TRAINING_DATA", this);
	}
	
	public String consumeAcks(){
		String a;
		synchronized(acks){
			if(!acks.isEmpty()){
				System.out.println("ABOLT: " + acks);
			}
			a = acks;
			acks = "";
		}
		return a;
	}
	
    @Override
    public void messageReceived(LCM lcm, String channel, LCMDataInputStream ins)
    {
       if(channel.equals("TRAINING_DATA")){
           training_data_t training;
            try{
            	training = new training_data_t(ins);
            } catch (IOException e) {
                e.printStackTrace();
                return;
            }

            for(int i=0; i<training.num_labels; i++){
                training_label_t tl = training.labels[i];
           	 	FeatureCategory cat = Features.getFeatureCategory(tl.cat.cat);
                BoltObject obj = null;
                if(tl.id != -1){
                	// Training example using an object
                	obj = Perception.getSingleton().getCurrentObjects().get(tl.id);
                	if(obj == null){
                		return;
                	}
                     ArrayList<Double> features = obj.getFeatures(cat);
                     if(features != null){
                         ClassifierManager.getSingleton().addDataPoint(cat, features, tl.label);
                     }
                } else if(tl.num_features > 0){
                	// Training example using a provided list of features
                	ArrayList<Double> features = new ArrayList<Double>();
                	for(Double d : tl.features){
                		features.add(d);
                	}
                	ClassifierManager.getSingleton().addDataPoint(cat, features, tl.label);
                }
            }
            synchronized(acks){
            	if(!acks.isEmpty()){
            		acks += ",";
            	}
            	acks += training.ack_nums;
            }
       }
    }
}
