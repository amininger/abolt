package abolt.kinect;

import java.awt.Color;
import java.awt.image.BufferedImage;
import java.awt.image.DataBufferByte;
import java.io.IOException;
import java.util.ArrayList;
import java.util.HashMap;

import lcm.lcm.LCM;
import lcm.lcm.LCMDataInputStream;
import lcm.lcm.LCMSubscriber;

import abolt.bolt.Bolt;
import abolt.classify.ColorFeatureExtractor;
import abolt.lcmtypes.kinect_status_t;
import april.vis.VisChain;
import april.vis.VisWorld;
import april.vis.VzImage;
import abolt.objects.*;

public class KinectCamera implements IBoltCamera, LCMSubscriber {
    final static int K_WIDTH = kinect_status_t.WIDTH;
    final static int K_HEIGHT = kinect_status_t.HEIGHT;

    private final static double darkThreshold = .4;

    // LCM
    static LCM lcm = LCM.getSingleton();

    private Segment segment;
    private kinect_status_t kinectData = null;
    private ArrayList<double[]> pointCloudData = null;

    public KinectCamera(){
    	//segment = new Segment((int)(KUtils.viewRegion.width),
        //        (int)(KUtils.viewRegion.height));
        segment = Segment.getSingleton();
    	lcm.subscribe("KINECT_STATUS", this);
    }

    /** Use the most recent frame from the kinect to extract a 3D point cloud
    and map it to the frame of the arm. **/
	public static ArrayList<double[]> extractPointCloudData(kinect_status_t kinectData)
	{
	    ArrayList<double[]> currentPoints = new ArrayList<double[]>();
	    if(kinectData == null){
	    	return currentPoints;
	    }

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
	
	public ArrayList<double[]> extractPointCloudData(){
		return extractPointCloudData(kinectData);
	}
	
	public BufferedImage getKinectImage(kinect_status_t kinectData){
    	if(kinectData == null){
    		return new BufferedImage(10, 10, BufferedImage.TYPE_3BYTE_BGR);
    	}
    	BufferedImage image = new BufferedImage(kinect_status_t.WIDTH, kinect_status_t.HEIGHT, BufferedImage.TYPE_3BYTE_BGR);
        byte[] buf = ((DataBufferByte)(image.getRaster().getDataBuffer())).getData();
        for (int i = 0; i < buf.length; i+=3) {
            int x = (i/3)%kinect_status_t.WIDTH;
            int y = (i/3)/kinect_status_t.WIDTH;
            if(KUtils.viewRegion.contains(x, y)){
                buf[i] = kinectData.rgb[i+2];   // B
                buf[i+1] = kinectData.rgb[i+1]; // G
                buf[i+2] = kinectData.rgb[i];   // R
            }
        }
        return image;
    }
	
	public void drawKinectData(double[][] viewXform, VisWorld.Buffer buffer){
    	BufferedImage background = getKinectImage(kinectData);
		double[] pt = new double[2];
		for(int y = (int)KUtils.viewRegion.getMinY(); y < KUtils.viewRegion.getMaxY(); y++){
			for(int x = (int)KUtils.viewRegion.getMinX(); x < KUtils.viewRegion.getMaxX(); x++){
	            int i = y*kinect_status_t.WIDTH + x;
	            int d = ((kinectData.depth[2*i+1]&0xff) << 8) |
	                    (kinectData.depth[2*i+0]&0xff);

                double[] pKinect = KUtils.getRegisteredXYZRGB(x,y, kinectData);
	            // double[] pKinect = KUtils.getXYZRGB(x, y, KUtils.depthLookup[d], kinectData);
	            double[] pixel = KUtils.getPixel(pKinect);
                Color c =  new Color((int)pKinect[3]);
                Color rc = new Color(c.getBlue(), c.getGreen(), c.getRed());
                background.setRGB((int)pixel[0], (int)pixel[1], rc.getRGB());
	        }
	    }
		buffer.addBack(new VisChain(viewXform, new VzImage(background, VzImage.FLIP)));
        buffer.setDrawOrder(-10);
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
                segment.segmentFrame(pointCloudData,
                                     KUtils.viewRegion.width,
                                     KUtils.viewRegion.height);
                synchronized(segment.objects){
                	HashMap<Integer, ObjectInfo> objInfoList = new HashMap<Integer, ObjectInfo>();
                	for(ObjectInfo info : segment.objects.values()){
                		ArrayList<Double> colorFeatures = ColorFeatureExtractor.getFeatures(info);
        	        	if(colorFeatures.get(0) > darkThreshold || colorFeatures.get(1) > darkThreshold ||
        	        			colorFeatures.get(2) > darkThreshold){
        	        		objInfoList.put(info.repID, info);
        	        	}
                    }
                	//Bolt.getObjectManager().updateObjects(objInfoList);
                    BoltObjectManager.getSingleton().updateObjects(objInfoList);
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
