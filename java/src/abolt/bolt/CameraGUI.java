package abolt.bolt;

import april.sim.Collisions;
import april.vis.*;
import april.jmat.*;
import april.jmat.geom.GRay3D;
import lcm.lcm.*;
import abolt.lcmtypes.*;
import abolt.objects.BoltObject;
import abolt.objects.BoltObjectManager;
import abolt.sim.SimSensable;
import abolt.kinect.*;
import abolt.classify.*;
import abolt.classify.Features.FeatureCategory;

import java.io.*;

import javax.swing.*;

import java.awt.*;
import java.util.*;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.awt.event.MouseEvent;
import java.awt.geom.AffineTransform;
import java.awt.image.*;


public class CameraGUI implements IBoltGUI, LCMSubscriber
{
	final static int K_WIDTH = kinect_status_t.WIDTH;
	final static int K_HEIGHT = kinect_status_t.HEIGHT;

	private VisWorld world;
	private VisLayer layer;
	private VisCanvas canvas;
	
	private BoltObject selectedObject = null;
    static LCM lcm = LCM.getSingleton();
    private kinect_status_t kinectData = null; 

	public CameraGUI(){
		world = new VisWorld();
		layer = new VisLayer(world);
		canvas = new VisCanvas(layer);

        // Set up camera
        layer.cameraManager.uiLookAt(new double[]{KUtils.viewRegion.getCenterX(),
                                                 kinect_status_t.HEIGHT-KUtils.viewRegion.getCenterY(),
                                                 KUtils.viewRegion.width},  //Position
                                    new double[]{KUtils.viewRegion.getCenterX(),
                                                 kinect_status_t.HEIGHT-KUtils.viewRegion.getCenterY(),
                                                 0}, // Lookat
                                    new double[]{0, 1, 0}, false); // Up
        layer.addEventHandler(new DisplayClickEventHandler());
    	lcm.subscribe("KINECT_STATUS", this);
	}

	@Override
	public VisCanvas getCanvas(){
		return canvas;
	}
	
	@Override
	public VisLayer getLayer(){
		return layer;
	}

	@Override
	public BoltObject getSelectedObject() {
		return selectedObject;
	}

	protected class DisplayClickEventHandler extends VisEventAdapter{
    	@Override
        public boolean mouseClicked(VisCanvas vc, VisLayer vl, VisCanvas.RenderInfo rinfo, GRay3D ray, MouseEvent e)
        {
    		double[] intersect = ray.intersectPlaneXY();
            double x = intersect[0];
            double y = K_HEIGHT - intersect[1];
            
            BoltObjectManager objManager = Bolt.getObjectManager();
            synchronized(objManager.objects){
            	for(BoltObject obj : objManager.objects.values()){
    				if(obj.getInfo().getProjectedBBox().contains(x, y)){
                    	selectedObject = obj;
    				}
        		}
            }
    		
            if(selectedObject != null){
            	System.out.println("CLICKED: " + selectedObject.getID());
            }
            return false;
        }
    }
	
    private BufferedImage getKinectImage(kinect_status_t kinectData){
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
	
    public void drawKinectData(){
        VisWorld.Buffer buffer = this.world.getBuffer("kinect");

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
		buffer.addBack(new VzImage(background, VzImage.FLIP));
        buffer.setDrawOrder(-10);
        buffer.swap();
    }

	@Override
	public void drawObjects(HashMap<Integer, BoltObject> objects) {
		double width = KUtils.viewRegion.getWidth() / 2 + 120;
    	double height = KUtils.viewRegion.getHeight() / 2 + 60;
    	VisWorld.Buffer buffer = world.getBuffer("objects");
    	buffer.setDrawOrder(10);

    	double theta = 0;
    	for(BoltObject obj : objects.values()){

    		// This code draws a white background behind objects to make the segmentation clear
//        		VzImage img = new VzImage(obj.getInfo().getImage());
//        		buffer.addBack(new VisChain(LinAlg.translate(obj.getBBox()[0][0], K_HEIGHT - projBBox.getMinY()), LinAlg.scale(1, -1, 1), img));
			
			Rectangle projBBox = obj.getInfo().getProjectedBBox();
    		VzRectangle rect = new VzRectangle(projBBox.getWidth(), projBBox.getHeight(), new VzLines.Style(Color.white, 3));
    		buffer.addBack(new VisChain(LinAlg.translate(projBBox.getCenterX(), K_HEIGHT - projBBox.getCenterY()), rect));		

    		double[] pixel = KUtils.getPixel(obj.getInfo().getCenter());
    		double x, y;
    		theta = Math.atan2(pixel[1] - KUtils.viewRegion.getCenterY(), pixel[0] - KUtils.viewRegion.getCenterX());
    		double vert = (Math.sin(theta) == 0) ? Double.MAX_VALUE : height / Math.abs(Math.sin(theta));
    		double horiz = (Math.cos(theta) == 0) ? Double.MAX_VALUE : width / Math.abs(Math.cos(theta));
    		if(vert < horiz){
    			x = KUtils.viewRegion.getCenterX() - 40 + Math.cos(theta) * vert;
    			y = KUtils.viewRegion.getCenterY() + 30 + Math.sin(theta) * vert;
    		} else {
    			x = KUtils.viewRegion.getCenterX() - 70 + Math.cos(theta) * horiz;
    			y = KUtils.viewRegion.getCenterY() + 30 + Math.sin(theta) * horiz;
    		}

    		String labelString = "";

    		String tf="<<monospaced,white,dropshadow=false>>";
    		labelString += String.format("%s%d\n", tf, obj.getID());
    		for(FeatureCategory cat : FeatureCategory.values()){
    			ConfidenceLabel label = obj.getLabels().getBestLabel(cat);
        		labelString += String.format("%s%s:%.2f\n", tf, label.getLabel(), label.getConfidence());
    		}
    		VzText text = new VzText(labelString);
            VisChain vch3 = new VisChain(LinAlg.translate(x, K_HEIGHT - y), LinAlg.scale(1.2), text);
            buffer.addBack(vch3);

    		VisVertexData line = new VisVertexData();
    		double[] objPos = new double[]{pixel[0],K_HEIGHT-pixel[1]};
    		double minDist = Double.MAX_VALUE;
    		double bestX = 0, bestY = 0;
    		for(int i = 0; i <= 1; i ++){
    			for(int j = 0; j <= 1; j ++){
    				double cx = x + i * 100;
    				double cy = y - j * 70;
    				double dist = LinAlg.normF(new double[]{cx - objPos[0], cy - objPos[1]});
    				if(dist < minDist){
    					minDist = dist;
    					bestX = cx;
    					bestY = cy;
    				}
    			}
    		}
    		line.add(objPos);
    		line.add(new double[]{bestX,K_HEIGHT-bestY});
    		buffer.addBack(new VzLines(line, VzLines.LINES,new VzLines.Style(Color.WHITE, 3)));
    	}
    	buffer.swap();
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
            drawKinectData();
        }
    }
	
    @Override
    public void drawVisObjects(String bufferName, ArrayList<VisObject> objects){
    	VisWorld.Buffer buffer = world.getBuffer(bufferName);
    	for(VisObject obj : objects){
    		buffer.addBack(obj);
    	}	
    	buffer.swap();
    }

}
