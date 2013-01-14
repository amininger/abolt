package abolt.bolt;

import java.awt.image.BufferedImage;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;

import abolt.classify.Features;
import abolt.kinect.KinectCamera;
import abolt.kinect.ObjectInfo;
import abolt.kinect.SimKinect;
import abolt.lcmtypes.observations_t;
import abolt.sim.SimBlock;
import abolt.sim.SimBoltObject;
import abolt.sim.SimLocation;
import april.sim.SimObject;
import april.util.PeriodicTasks;
import april.util.TimeUtil;

public class Perception {
	static Perception singleton = null;
	public static Perception getSingleton(){
		return singleton;
	}
	public static void Initialize(){
		singleton = new Perception();
	}
	
	private SimKinect simKinect;
	private KinectCamera kinectCamera;
	
	private HashMap<Integer, BoltObject> currentObjects = new HashMap<Integer, BoltObject>();

    PeriodicTasks sendTasks = new PeriodicTasks(2);
    private static final double UPDATE_RATE = .2;
	
	public Perception(){		
		simKinect = new SimKinect(400, 300, BoltSimulator.getSingleton());
		kinectCamera = new KinectCamera(BoltSimulator.getSingleton().getVisWorld());
		sendTasks.addFixedDelay(new UpdatePerceptionTask(), UPDATE_RATE);
		sendTasks.setRunning(true);
	}
	
	public HashMap<Integer, BoltObject> getCurrentObjects(){
		return currentObjects;
	}
	
	class UpdatePerceptionTask implements PeriodicTasks.Task{
		public UpdatePerceptionTask() {}

        public void run(double dt)
        {
        	simKinect.update();
    		HashMap<Integer, BoltObject> objects = new HashMap<Integer, BoltObject>();
    		addSimulatedObjects(objects);
    		addPerceivedObjects(objects);
    		for(BoltObject obj : objects.values()){
    			Features.determineFeatures(obj);
    		}
    		currentObjects = objects;
        }
	}
	
	private void addSimulatedObjects(HashMap<Integer, BoltObject> objects){
		// Get simulated data
		ArrayList<SimObject> simObjects = BoltSimulator.getSingleton().getWorld().objects;
		synchronized(simObjects){
			for(SimObject obj : simObjects){
				if(obj instanceof SimBoltObject && !((SimBoltObject)obj).getVisible()){
					//continue;
				}
				if(obj instanceof SimBlock){
					// Get points of the object
					ArrayList<double[]> points = simKinect.traceObject((SimBlock)obj);
					if(points == null){
						continue;
					}
					
					// Project the points onto a 2D image
					BufferedImage projection = Features.getImage(simKinect, points);
					
					// Transform the points into world coordinates
					for(int i = 0; i < points.size(); i++){
						points.set(i, simKinect.getWorldCoords(points.get(i)));
					}
					
					// Create a new bolt object and add it to the list
					BoltObject boltObject = new BoltObject(((SimBlock)obj).getID(), points, projection);
					boltObject.sourceObject = (SimBlock)obj;
					objects.put(((SimBlock)obj).getID(), boltObject);
				} else if(obj instanceof SimLocation){
					objects.put(((SimLocation)obj).getID(), (new BoltObject((SimLocation)obj)));
				}
			}
		}
	}
	
	private void addPerceivedObjects(HashMap<Integer, BoltObject> objects){
		HashSet<ObjectInfo> objectInfos = kinectCamera.getLastObjects();
		synchronized(objectInfos){
			for(ObjectInfo obj : objectInfos){
				ArrayList<double[]> points = obj.points;

				// Project the points onto a 2D image
				BufferedImage projection = Features.getImage(kinectCamera, points);
				
				// Transform the points into world coordinates
				for(int i = 0; i < points.size(); i++){
					points.set(i, kinectCamera.getWorldCoords(points.get(i)));
				}
				
				// Create a new bolt object and add it to the list
				objects.put(obj.repID, new BoltObject(obj.repID, points, projection));
			}
		}
	}
}
