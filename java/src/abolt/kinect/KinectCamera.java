package abolt.kinect;

import java.io.IOException;
import java.util.ArrayList;
import java.util.HashMap;

import lcm.lcm.LCM;
import lcm.lcm.LCMDataInputStream;
import lcm.lcm.LCMSubscriber;
import abolt.bolt.Bolt;
import abolt.classify.ColorFeatureExtractor;
import abolt.lcmtypes.kinect_status_t;
import abolt.objects.WorldBoltObject;

public class KinectCamera implements IBoltCamera, LCMSubscriber {
    final static int K_WIDTH = kinect_status_t.WIDTH;
    final static int K_HEIGHT = kinect_status_t.HEIGHT;
    
    private final static double darkThreshold = .4;
    
    // LCM
    static LCM lcm = LCM.getSingleton();

    private HashMap<Integer, WorldBoltObject> objects;
    private Segment segment;
    private kinect_status_t kinectData = null;
    private ArrayList<double[]> pointCloudData = null;
    
    public KinectCamera(){
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
                synchronized(segment.objects){
                	HashMap<Integer, ObjectInfo> objInfoList = new HashMap<Integer, ObjectInfo>();
                	for(ObjectInfo info : segment.objects.values()){
                		ArrayList<Double> colorFeatures = ColorFeatureExtractor.getFeatures(info);
        	        	if(colorFeatures.get(0) > darkThreshold || colorFeatures.get(1) > darkThreshold ||
        	        			colorFeatures.get(2) > darkThreshold){
        	        		objInfoList.put(info.repID, info);
        	        	}   
                    }
                	Bolt.getObjectManager().updateObjects(objInfoList);
                }
            }
        }
    }

	@Override
	public int[] getPixel(double[] cameraPt) {
		double[] pixel = KUtils.getPixel(cameraPt);
		return new int[]{(int) Math.round(pixel[0]), (int) Math.round(pixel[1])};
	}

	@Override
	public double[] getWorldCoords(double[] cameraPt) {
		return KUtils.getWorldCoordinates(cameraPt);
	}
}
