package abolt.objects;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Timer;
import java.util.TimerTask;

import abolt.bolt.Bolt;
import abolt.classify.ClassifierManager;
import abolt.lcmtypes.object_data_t;

public class SimObjectManager implements IObjectManager {
    
    private HashMap<Integer, BoltObject> objects;
    private Timer updateObjectsTimer;
    private static final int UPDATE_OBJECTS_RATE = 2; // # updates per second
    
    public SimObjectManager(){
    	objects = new HashMap<Integer, BoltObject>();
    	class UpdateObjectsTask extends TimerTask{
			public void run() {
				updateObjects();
			}
    	}
    	updateObjectsTimer = new Timer();
    	updateObjectsTimer.schedule(new UpdateObjectsTask(), 1000, 1000/UPDATE_OBJECTS_RATE);
    }

	@Override
	public object_data_t[] getObjectData() {
		synchronized(objects){
			ArrayList<object_data_t> objData = new ArrayList<object_data_t>();
			for(BoltObject obj : objects.values()){
				object_data_t data = obj.getData();
				if(data != null){
					objData.add(obj.getData());
				}
			}
			return objData.toArray(new object_data_t[0]);
		}
	}

	@Override
	public HashMap<Integer, BoltObject> getObjects() {
		return objects;
	}

	@Override
	public void addObject(BoltObject obj) {
		synchronized(objects){
			objects.put(obj.getID(), obj);
		}
	}
	
	private void updateObjects(){
		synchronized(objects){
			ClassifierManager cm = Bolt.getClassifierManager();
			for(BoltObject obj : objects.values()){
				cm.updateObject(obj);
			}
		}
	}
}
