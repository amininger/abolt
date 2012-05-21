package abolt.objects;

import java.awt.Rectangle;
import java.io.IOException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Set;

import abolt.bolt.Bolt;
import abolt.classify.ClassifierManager;
import abolt.classify.ColorFeatureExtractor;
import abolt.kinect.KUtils;
import abolt.kinect.ObjectInfo;
import abolt.kinect.Segment;
import abolt.lcmtypes.kinect_status_t;
import abolt.lcmtypes.object_data_t;
import lcm.lcm.LCM;
import lcm.lcm.LCMDataInputStream;
import lcm.lcm.LCMSubscriber;

public class WorldObjectManager implements IObjectManager, LCMSubscriber {
    final static int K_WIDTH = kinect_status_t.WIDTH;
    final static int K_HEIGHT = kinect_status_t.HEIGHT;
    
    private final static double darkThreshold = .4;
    
    // LCM
    static LCM lcm = LCM.getSingleton();

    private HashMap<Integer, WorldBoltObject> objects;
    private Segment segment;
    private kinect_status_t kinectData = null;
    private ArrayList<double[]> pointCloudData = null;
    
    public WorldObjectManager(){
    	objects = new HashMap<Integer, WorldBoltObject>();
    	segment = new Segment((int)(KUtils.viewRegion.width),
                (int)(KUtils.viewRegion.height));
    	lcm.subscribe("KINECT_STATUS", this);
    }
    
    public Segment getSegment(){
    	return segment;
    }
    
    /** Use the most recent frame from the kinect to extract a 3D point cloud
    and map it to the frame of the arm. **/
	private ArrayList<double[]> extractPointCloudData(kinect_status_t kinectData)
	{
	    ArrayList<double[]> currentPoints = new ArrayList<double[]>();
	
	    for (int y = (int) KUtils.viewRegion.getMinY(); y < KUtils.viewRegion.getMaxY(); y++) {
	        for (int x = (int) KUtils.viewRegion.getMinX(); x < KUtils.viewRegion.getMaxX(); x++) {
	            int i = y * kinect_status_t.WIDTH + x;
	            int d = ((kinectData.depth[2 * i + 1] & 0xff) << 8)
	                | (kinectData.depth[2 * i + 0] & 0xff);
	            double[] pKinect = KUtils.getRegisteredXYZRGB(x,y, kinectData);
	
	            // Disabled to switch to registered view
	            // KUtils.getXYZRGB(x, y, KUtils.depthLookup[d],
	            //                                 kinectData);
	            currentPoints.add(pKinect);
	        }
	    }
	    return currentPoints;
	}
    
	public void updateObjects(HashMap<Integer, ObjectInfo> objectInfo) {
		synchronized(objects){
	        Set<Integer> objsToRemove = new HashSet<Integer>();
	        for (Integer id : objects.keySet()) {
	        	// Start out assuming we will remove all the objects
	            objsToRemove.add(id);
	        }
	        
	        
	        for (ObjectInfo info : objectInfo.values()) {
	        	ArrayList<Double> colorFeatures = ColorFeatureExtractor.getFeatures(info);
	        	if(colorFeatures.get(0) < darkThreshold && colorFeatures.get(1) < darkThreshold &&
	        			colorFeatures.get(2) < darkThreshold){
	        		continue;
	        	}       	
	        	
	        	int id = info.repID;
	        	WorldBoltObject bObject;
	            if (objects.containsKey(id)) {
	            	// The object already exists
	                bObject = objects.get(id);
	                objsToRemove.remove(id);
	            } else {
	            	// Create a new object
	                bObject = new WorldBoltObject(id);
	                objects.put(id, bObject);
	            }
	            bObject.updateObject(info);
	            Bolt.getClassifierManager().updateObject(bObject);
	        }
	
	        for (Integer id : objsToRemove) {
	            objects.remove(id);
	        }
        }
        
        Bolt.getBoltGUI().drawObjects((HashMap)objects);
	}
	
	@Override
    public void messageReceived(LCM lcm, String channel, LCMDataInputStream ins)
    {
		if(channel.equals("KINECT_STATUS")){
            try {
                kinectData = new kinect_status_t(ins);
            } catch (IOException e) {
                e.printStackTrace();
                return;
            }
            pointCloudData = extractPointCloudData(kinectData);
            if(pointCloudData.size() > 0){
                segment.segmentFrame(pointCloudData);
                updateObjects(segment.objects);
            }
        }
    }

	@Override
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

	@Override
	public HashMap<Integer, BoltObject> getObjects() {
		return (HashMap)objects;
	}

	@Override
	public void addObject(BoltObject obj) {
		if(obj instanceof WorldBoltObject){
			synchronized(objects){
				objects.put(obj.getID(), (WorldBoltObject)obj);
			}
		}
	}

}
