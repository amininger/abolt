package abolt.objects;

import java.util.*;

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

    private BoltObjectManager(){
    	objects = new HashMap<Integer, BoltObject>();
    }

	public void updateObjects(HashMap<Integer, ObjectInfo> objectInfo) {
		synchronized(objects){
	        Set<Integer> objsToRemove = new HashSet<Integer>();
	        for (Integer id : objects.keySet()) {
	        	// Start out assuming we will remove all the objects
	            objsToRemove.add(id);
	        }

	        for (ObjectInfo info : objectInfo.values()) {
                double[] c = info.center;
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
	        }

	        for (Integer id : objsToRemove) {
	            objects.remove(id);
	        }
		}
	}

	public void addObject(BoltObject obj) {
		synchronized(objects){
			objects.put(obj.getID(), obj);
		}
	}
}
