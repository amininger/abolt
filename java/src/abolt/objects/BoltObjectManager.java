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
	            ClassifierManager.getSingleton().updateObject(obj);
	        }

	        for (Integer id : objsToRemove) {
	        	if(BoltArmController.Singleton == null ||
	        			id != BoltArmController.Singleton.grabbedObject()){
		            objects.remove(id);
	        	}
	        }
		}
	}

	public object_data_t[] getObjectData() {
		ArrayList<object_data_t> objData = new ArrayList<object_data_t>();
		synchronized(objects){
			for(BoltObject obj : objects.values()){
				object_data_t data = obj.getData();
				if(data != null){
					objData.add(obj.getData());
				}
			}
			return objData.toArray(new object_data_t[0]);
		}
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
