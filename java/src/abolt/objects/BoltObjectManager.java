package abolt.objects;

import java.util.*;

import abolt.arm.BoltArmController;
import abolt.bolt.*;
import abolt.kinect.*;
import abolt.lcmtypes.object_data_t;
import abolt.classify.*;

/** Maintains state about the objects in the world */
public class BoltObjectManager {

    static BoltObjectManager singleton = null;
    public static BoltObjectManager getSingleton()
    {
        if (singleton == null) {
            singleton = new BoltObjectManager();
        }
        return singleton;
    }

    public HashMap<Integer, BoltObject> objects;
    private HashSet<ObjectEffector> effectors;

    private BoltObjectManager(){
    	objects = new HashMap<Integer, BoltObject>();
    	effectors = new HashSet<ObjectEffector>();
    }

	public void updateObjects(HashMap<Integer, ObjectInfo> objectInfo) {
		synchronized(objects){
	        Set<Integer> objsToRemove = new HashSet<Integer>();
	        for (Integer id : objects.keySet()) {
	        	// Start out assuming we will remove all the objects
	            objsToRemove.add(id);
	        }

	        for (ObjectInfo info : objectInfo.values()) {
                int id = info.repID;
	        	BoltObject obj;
	            if (objects.containsKey(id)) {
	            	// The object already exists
	            	obj = objects.get(id);
	                objsToRemove.remove(id);
	            } else {
	            	// Create a new object
	            	obj = new BoltObject(id);
	                objects.put(id, obj);
	            }
	            obj.updateObject(info);
                //ClassifierManager.getSingleton().updateObject(obj);
	        }

	        for (Integer id : objsToRemove) {
		       objects.remove(id);
	        }
	        
	        for(ObjectEffector effector : effectors){
	        	for(BoltObject obj : objects.values()){
	        		effector.effect(obj);
	        	}
	        }
		}
	}
	
	public void addEffector(ObjectEffector effector){
		effectors.add(effector);
	}
	
	public void removeEffector(ObjectEffector effector){
		effectors.remove(effector);
	}

	public void addObject(BoltObject obj) {
		synchronized(objects){
			objects.put(obj.getID(), obj);
		}
	}
	
	public void removeObject(int id){
		synchronized(objects){
			objects.remove(id);
		}
	}
}
