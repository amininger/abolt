package abolt.bolt;

import java.util.ArrayList;
import java.util.HashMap;

import lcm.lcm.LCM;
import abolt.classify.ClassifierManager;
import abolt.lcmtypes.object_data_t;
import abolt.lcmtypes.observations_t;
import april.util.PeriodicTasks;
import april.util.TimeUtil;

public class LCMBroadcaster {
	static LCMBroadcaster singleton = null;
	public static LCMBroadcaster getSingleton(){
		return singleton;
	}
	
	public static void Initialize(){
		singleton = new LCMBroadcaster();
	}

    static LCM lcm = LCM.getSingleton();
    PeriodicTasks sendTasks = new PeriodicTasks(2);
    
    private static final double OBS_RATE = .5;
	
	public LCMBroadcaster(){
		sendTasks.addFixedDelay(new SendObservationTask(), OBS_RATE);
	}
	
	public void start(){
		sendTasks.setRunning(true);
	}
	
	class SendObservationTask implements PeriodicTasks.Task{
		public SendObservationTask() {}

        public void run(double dt)
        {
            observations_t obs = new observations_t();
            obs.utime = TimeUtil.utime();
            
            obs.ack_nums = LCMReceiver.getSingleton().consumeAcks();
            obs.click_id = BoltSimulator.getSingleton().getSelectedId();
            
            obs.observations = getObjectData();
            obs.nobs = obs.observations.length;

            lcm.publish("OBSERVATIONS",obs);
        }
	}


    /** Build up the object_data_t describing the observed objects
     *  in the world. Runs classifiers on the objects and builds
     *  the appropriate lcmtypes to return.
     */
    public object_data_t[] getObjectData()
    {
    	HashMap<Integer, BoltObject> objects = Perception.getSingleton().getCurrentObjects();
    	ArrayList<object_data_t> objectDatas = new ArrayList<object_data_t>();

        for (BoltObject bo: objects.values()) {
        	if(!bo.isVisible()){
        		continue;
        	}
        	object_data_t obj_data = ClassifierManager.getSingleton().getObjectData(bo);
        	if(obj_data != null){
        		objectDatas.add(obj_data);
        	}
        }

        return objectDatas.toArray(new object_data_t[objectDatas.size()]);
    }

}
