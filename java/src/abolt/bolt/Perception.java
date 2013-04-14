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
	/// Singleton
	private static Perception singleton = null;
	public static Perception getSingleton(){
		return singleton;
	}
	
	/// Consts
    private static final double UPDATE_RATE = .1;
    
    /// Variables
	private SimKinect simKinect;
	private KinectCamera kinectCamera;
	
	private HashMap<Integer, BoltObject> currentObjects = new HashMap<Integer, BoltObject>();

    PeriodicTasks sendTasks = new PeriodicTasks(2);
    
    long time = 0;
	
    /// Constructor
	public Perception(){		
		singleton = this;
		
		simKinect = new SimKinect(400, 300, BoltSimulator.getSingleton());
		kinectCamera = new KinectCamera(BoltSimulator.getSingleton().getVisWorld());
		sendTasks.addFixedDelay(new UpdatePerceptionTask(), UPDATE_RATE);
		sendTasks.setRunning(true);
	}
	
	/// Accessors/Mutators
	public SimKinect getSimKinect(){
		return simKinect;
	}
	
	public KinectCamera getKinect(){
		return kinectCamera;
	}

	public HashMap<Integer, BoltObject> getCurrentObjects(){
		return currentObjects;
	}
	
	/// Methods
	
	/***
	 * This is the main processing cycle for perception
	 * First add the objects from both the simulator and actual kinect
	 * Then determine the features for those objects
	 */
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
						double[] pt = simKinect.getWorldCoords(points.get(i));
						points.set(i, new double[]{pt[0], pt[1], pt[2], points.get(i)[3]});
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
				ArrayList<double[]> worldPts = new ArrayList<double[]>(points.size());

				// Project the points onto a 2D image
				BufferedImage projection = Features.getImage(kinectCamera, points);
				
				// Transform the points into world coordinates
				for(int i = 0; i < points.size(); i++){
					double[] pt = kinectCamera.getWorldCoords(points.get(i));
					worldPts.add(new double[]{pt[0], pt[1], pt[2], points.get(i)[3]});
				}
				
				// Create a new bolt object and add it to the list
				BoltObject bo = new BoltObject(obj.repID, worldPts, projection);
				objects.put(obj.repID, bo);
			}
		}
	}
}
