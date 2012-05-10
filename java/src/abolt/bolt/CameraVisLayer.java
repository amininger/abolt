package abolt.bolt;

import april.vis.*;
import april.jmat.*;
import april.jmat.geom.GRay3D;
import lcm.lcm.*;
import abolt.lcmtypes.*;
import abolt.kinect.*;
import abolt.ispy.*;
import abolt.classify.*;

import java.io.*;
import javax.swing.*;

import java.awt.*;
import java.util.*;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.awt.event.MouseEvent;
import java.awt.geom.AffineTransform;
import java.awt.image.*;

/* To Do:
 *  - Add features for shape recognition to FeatureVec
 */

public class CameraVisLayer extends VisLayer
{
	final static int K_WIDTH = kinect_status_t.WIDTH;
	final static int K_HEIGHT = kinect_status_t.HEIGHT;

	private final Bolt bolt;
	private VisCanvas canvas;
	private boolean drawSegmentation = false;


	public CameraVisLayer(VisWorld world, Bolt bolt){
		super(world);
		this.bolt = bolt;
		canvas = new VisCanvas(this);

        // Set up camera
        this.cameraManager.uiLookAt(new double[]{Bolt.viewRegion.getCenterX(),
                                                 kinect_status_t.HEIGHT-Bolt.viewRegion.getCenterY(),
                                                 Bolt.viewRegion.width},  //Position
                                    new double[]{Bolt.viewRegion.getCenterX(),
                                                 kinect_status_t.HEIGHT-Bolt.viewRegion.getCenterY(),
                                                 0}, // Lookat
                                    new double[]{0, 1, 0}, false); // Up
        this.addEventHandler(new DisplayClickEventHandler());
	}

	public VisCanvas getCanvas(){
		return canvas;
	}
	
	public void setDrawSegmentation(boolean shouldDrawSeg){
		this.drawSegmentation = shouldDrawSeg;
	}

	protected class DisplayClickEventHandler extends VisEventAdapter{
    	@Override
        public boolean mouseClicked(VisCanvas vc, VisLayer vl, VisCanvas.RenderInfo rinfo, GRay3D ray, MouseEvent e)
        {
            double[] intersect = ray.intersectPlaneXY();
            double x = intersect[0];
            double y = K_HEIGHT - intersect[1];
            bolt.mouseClicked(x, y);
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
            if(Bolt.viewRegion.contains(x, y)){
                buf[i] = kinectData.rgb[i+2];   // B
                buf[i+1] = kinectData.rgb[i+1]; // G
                buf[i+2] = kinectData.rgb[i];   // R
            }
        }
        return image;
    }

    public void drawObjects(VisWorld.Buffer buffer, Map<Integer, SpyObject> objects){
    	double width = Bolt.viewRegion.getWidth() / 2 + 80;
    	double height = Bolt.viewRegion.getHeight() / 2 + 60;

    	double theta = 0;
    	for(SpyObject obj : objects.values()){
    		if(drawSegmentation){
        		VzImage img = new VzImage(obj.object.getImage());
        		buffer.addBack(new VisChain(LinAlg.translate(obj.bbox.getMinX(), K_HEIGHT - obj.bbox.getMinY()), LinAlg.scale(1, -1, 1), img));
    		}

    		double x, y;
    		theta = Math.atan2(obj.bbox.getCenterY() - Bolt.viewRegion.getCenterY(), obj.bbox.getCenterX() - Bolt.viewRegion.getCenterX());
    		double vert = (Math.sin(theta) == 0) ? Double.MAX_VALUE : height / Math.abs(Math.sin(theta));
    		double horiz = (Math.cos(theta) == 0) ? Double.MAX_VALUE : width / Math.abs(Math.cos(theta));
    		if(vert < horiz){
    			x = Bolt.viewRegion.getCenterX() - 40 + Math.cos(theta) * vert;
    			y = Bolt.viewRegion.getCenterY() + 30 + Math.sin(theta) * vert;
    		} else {
    			x = Bolt.viewRegion.getCenterX() - 40 + Math.cos(theta) * horiz;
    			y = Bolt.viewRegion.getCenterY() + 30 + Math.sin(theta) * horiz;
    		}

    		String labelString = "";

    		String tf="<<monospaced,white,dropshadow=false>>";
    		labelString += String.format("%s%d\n", tf, obj.id);
    		labelString += String.format("%s%s:%.2f\n", tf, obj.getColor(), obj.getColorConfidence());
    		labelString += String.format("%s%s:%.2f\n", tf, obj.getShape(), obj.getShapeConfidence());
    		labelString += String.format("%s%s:%.2f\n", tf, obj.getSize(), obj.getSizeConfidence());
    		VzText text = new VzText(labelString);
            VisChain vch3 = new VisChain(LinAlg.translate(x, K_HEIGHT - y), LinAlg.scale(1.2), text);
            buffer.addBack(vch3);

    		VisVertexData line = new VisVertexData();
    		double[] objPos = new double[]{obj.bbox.getCenterX(),K_HEIGHT-obj.bbox.getCenterY()};
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

    		VzRectangle rect = new VzRectangle(obj.bbox.getWidth(), obj.bbox.getHeight(), new VzLines.Style(obj.boxColor, 3));
    		buffer.addBack(new VisChain(LinAlg.translate(obj.bbox.getCenterX(), K_HEIGHT - obj.bbox.getCenterY()), rect));
    	}
    }

    public void drawScene(kinect_status_t kinectData, Map<Integer, SpyObject> objects){
        VisWorld.Buffer worldBuffer = this.world.getBuffer("displayImage");

    	BufferedImage background = getKinectImage(kinectData);
		double[] pt = new double[2];
		for(int y = (int)Bolt.viewRegion.getMinY(); y < Bolt.viewRegion.getMaxY(); y++){
			for(int x = (int)Bolt.viewRegion.getMinX(); x < Bolt.viewRegion.getMaxX(); x++){
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
        worldBuffer.addBack(new VzImage(background, VzImage.FLIP));
        worldBuffer.setDrawOrder(-10);
        worldBuffer.swap();

        VisWorld.Buffer textBuffer = this.world.getBuffer("textInfo");
        drawObjects(textBuffer, objects);
        textBuffer.setDrawOrder(10);
        textBuffer.swap();
    }
}
