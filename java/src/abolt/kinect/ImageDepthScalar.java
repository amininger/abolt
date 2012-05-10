package abolt.kinect;

import april.vis.*;
import april.jmat.*;
import april.jmat.geom.GRay3D;
import april.util.*;

import lcm.logging.*;
import abolt.kinect.KinectCalibrator.ClickEventHandler;
import abolt.lcmtypes.*;


import java.io.*;
import javax.swing.*;
import java.awt.*;
import java.awt.event.MouseEvent;
import java.awt.image.*;


class ImageDepthScalar // implements LCMSubscriber
{
	private enum Mode {ORIGIN, X, Y, TEST};
    final int FRAME_WIDTH = 800;
    final int FRAME_HEIGHT = 600;
    final int MIN_POINTS = 250;
    
    final int KINECT_WIDTH = kinect_status_t.WIDTH;
    final int KINECT_HEIGHT = kinect_status_t.HEIGHT;
    
    float[] depthLookUp = KUtils.createDepthMap();

    int[] xDirs = new int[]{-1, 0, 1, -1};
    int[] yDirs = new int[]{-1, -1, -1, 0};

    VisWorld visWorld;
    VisLayer imageLayer;
    Log log;
    
    String calibFilename;
    
    Mode curMode = Mode.ORIGIN;
    double[] originLocation = null;
    double[] xLocation = null;
    double[] yLocation = null;
    double[] testLocation = null;

    // The most recently accessed kinect status
    kinect_status_t ks = null;

    public ImageDepthScalar(GetOpt opt)
    {
    	// Calibration variables
        calibFilename = "kinect.calib";

        try{
            log = new Log(opt.getString("log"), "r");
        }catch(Exception e){
            System.err.println("Error: "+e);
        }
        
        // Setup Frame
        JFrame frame = new JFrame("Calibrate Kinect");
        frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        frame.setLayout(new BorderLayout());
        
        // Initialize the image frame and canvas
        visWorld = new VisWorld();
        
        imageLayer = new VisLayer(visWorld);
        VisCanvas visCanvas = new VisCanvas(imageLayer);
        
        //Set up initial camera view
        imageLayer.cameraManager.uiLookAt(new double[] {KINECT_WIDTH/2, KINECT_HEIGHT/2, 400},// Camera position
                                  new double[] {KINECT_WIDTH/2, KINECT_HEIGHT/2, 0.0},// Point looking at
                                  new double[] {0.0, 1.0, 0.0},// Up
                                  false);

    	imageLayer.addEventHandler(new ClickEventHandler());

        getKinectData();
        redrawImage();

        frame.add(visCanvas, BorderLayout.CENTER);

        // Finalize JFrame
        frame.setSize(FRAME_WIDTH, FRAME_HEIGHT);
        frame.setVisible(true);
    }
    
    protected class ClickEventHandler extends VisEventAdapter{
    	public boolean mouseClicked(VisCanvas vc, VisLayer vl, VisCanvas.RenderInfo rinfo, GRay3D ray, MouseEvent e)
        {
    		double[] intersect = ray.intersectPlaneXY();
    		double x = intersect[0];
    		double y = intersect[1];
    		double[] location = new double[3];
    		
    		if(x < 1 || x >= KINECT_WIDTH-1 || y < 1 || y >= KINECT_HEIGHT-1){
    			// Out of bounds
    			return false;
    		}
    		
    		
            return false;
        }
    }

    
    private String v2s(double[] v){
    	return String.format("(%f, %f, %f)", v[0], v[1], v[2]);
    }
    
    /** Takes a point in image coordinates and returns a point in the space of the kinect **/
    public double[] getKinectPoint(int x, int y){
		int index = y*KINECT_WIDTH + x;
		int depth = ((ks.depth[2*index+1]&0xff) << 8) | (ks.depth[2*index+0]&0xff);
		double[] xyzrgb = KUtils.getXYZRGB(x, y, depthLookUp[depth], ks);
    	
    	// System.out.println("Depth: " + depthLookUp[depth]);
    	//System.out.println(String.format("(%f, %f, %f)", xyzrgb[0], xyzrgb[1], xyzrgb[2]));
    	return new double[]{xyzrgb[0], xyzrgb[1], xyzrgb[2]};
    }

    /** Load in the next frame from the log..**/
    public void getKinectData()
    {    	
        try{
            ks = new kinect_status_t(log.readNext().data);
        } catch(Exception e){
            System.err.println(e.getMessage());
        }
    }
    
//    double maxDelta = 0;
//	int height = data.length;
//	int width = data[0].length;
//	for(int dX = -scale; dX <= scale; dX +=scale){
//    	for(int dY = -scale; dY <= scale; dY +=scale){
//			int xp = x + dX;
//			int yp = y + dY;
//	    	if(xp >= 0 && xp < width && yp >= 0 && yp < height){
//	    		double delta = Math.abs(data[yp][xp][channel] - data[y][x][channel]);
//	    		maxDelta = (maxDelta < delta ? delta : maxDelta);
//	    	}
//			
//		}
//	}
//	return maxDelta;
    
    public double getDelta(double[][][] data, int x, int y, int scale, int channel){
    	double min = data[y][x][channel];
    	double max = data[y][x][channel];
    	int height = data.length;
    	int width = data[0].length;
    	for(int dX = -scale; dX <= scale; dX+=scale){
    		for(int dY = -scale; dY <= scale; dY+=scale){
    			int xp = x + dX;
    			int yp = y + dY;
    	    	if(xp >= 0 && xp < width && yp >= 0 && yp < height){
    	    		if(data[yp][xp][channel] < min){
    	    			min = data[yp][xp][channel];
    	    		} else if(data[yp][xp][channel] > max){
    	    			max = data[yp][xp][channel];
    	    		}
    	    	}
    			
    		}
    	}
    	return max - min;
    }

    
    public int min(int a, int b){
    	return (a < b ? a : b);
    }
    
    public int max(int a, int b){
    	return (a > b ? a : b);
    }
    
    public double[][][] getImageData(BufferedImage image, int width, int height){
    	double[][][] imgData = new double[height][width][];
        for(int x = 0; x < image.getWidth(); x++){
        	for(int y = 0; y < image.getHeight(); y++){
        		if(x < width && y < height){
            		imgData[y][x] = getKinectPoint(x, y);
        		}
        	}
        }
        return imgData;
    }
    
    public double[][][] getColorData(BufferedImage image, int width, int height){
        double[][][] colorData = new double[height][width][];
        for(int x = 0; x < image.getWidth(); x++){
        	for(int y = 0; y < image.getHeight(); y++){
        		Color color = new Color(image.getRGB(x,  y));
        		double lum = .3*color.getRed() + .59*color.getGreen() + .11 * color.getBlue();
        		if(x < width && y < height){
            		colorData[y][x] = new double[]{color.getRed(), color.getGreen(), color.getBlue(), lum};
        		}
        		//image.setRGB(x, y, new Color((int)lum, (int)lum, (int)lum).getRGB());
        	}
        }
        return colorData;
    }
    
    public BufferedImage getBufferedImage(){
    	BufferedImage image = new BufferedImage(KINECT_WIDTH, KINECT_HEIGHT, BufferedImage.TYPE_3BYTE_BGR);
        byte[] buf = ((DataBufferByte)(image.getRaster().getDataBuffer())).getData();
        for (int i = 0; i < buf.length; i+=3) {
            buf[i] = ks.rgb[i+2];	// B
            buf[i+1] = ks.rgb[i+1];	// G
            buf[i+2] = ks.rgb[i];	// R
        }
        return image;
    }
    
    /** Draws the image on the screen with circles where the user has clicked **/
    public void redrawImage(){
    	BufferedImage image = getBufferedImage();
        
        int testWidth = 600;
        int testHeight = 400;
        
        double[][][] colorData = getColorData(image, image.getWidth(), image.getHeight());
        
        for(int x = 100; x < image.getWidth() -100; x++){
    		for(int y = 75; y < image.getHeight() - 75; y++){  
            	float intensity;    
        		double maxDelta = 0;
        		for(int channel = 0; channel < 3; channel++){
        			double delta = getDelta(colorData, x, y, 1, channel)/255;
        			maxDelta = (maxDelta < delta ? delta : maxDelta);
        		}
        		//intensity = (float)maxDelta;
        		intensity = (float)getDelta(colorData, x, y, 1, 3)/255;
        		double threshold = .12;
        		double scale = 100;
        		//intensity = (float)(1/(1 + Math.exp(-(intensity-threshold)*scale)));

        		image.setRGB(x, y, (new Color(intensity, intensity, intensity)).getRGB());	
        	}
        }

        // Render the kinect image
        VisWorld.Buffer visBuffer = visWorld.getBuffer("backgroundImage");
        visBuffer.addBack(new VzImage(image, VzImage.FLIP));
        
        visBuffer.swap();
    }
   

    public static void main(String[] args)
    {
        // Define possible commandline arguments
        GetOpt opts = new GetOpt();
        opts.addBoolean('h', "help", false, "Show this help screen");
        opts.addString('l', "log", "stdout", "Log file to get the image from");

        if (!opts.parse(args)) {
            System.err.println("ERR: Opts error - " + opts.getReason());
            System.exit(1);
        }
        if (opts.getBoolean("help")) {
            opts.doHelp();
            System.exit(1);
        }

        //segment = new Segment(da, true);
        ImageDepthScalar kc = new ImageDepthScalar(opts);
    }
}
