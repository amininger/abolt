package abolt.kinect;


import lcm.lcm.*;
import lcm.logging.*;

import java.io.*;
import javax.swing.*;
import java.awt.*;
import java.awt.event.MouseEvent;
import java.awt.image.*;

import april.vis.*;
import april.config.*;
import april.jmat.*;
import april.jmat.geom.*;
import april.util.*;

import abolt.lcmtypes.*;


class KinectCalibrator // implements LCMSubscriber
 implements LCMSubscriber
{
	private enum Mode {ORIGIN, X, Y, TEST};
    final int FRAME_WIDTH = 800;
    final int FRAME_HEIGHT = 600;
    final int MIN_POINTS = 250;

    final int KINECT_WIDTH = kinect_status_t.WIDTH;
    final int KINECT_HEIGHT = kinect_status_t.HEIGHT;

    float[] depthLookUp = KUtils.createDepthMap();

    LCM myLCM;
    VisWorld visWorld;
    VisLayer imageLayer;

    String calibFilename;

    Mode curMode = Mode.ORIGIN;
    double[] originLocation = null;
    double[] xLocation = null;
    double[] yLocation = null;
    double[] testLocation = null;

    static double x_offset = 0;

    // The most recently accessed kinect status
    kinect_status_t ks = null;

    public KinectCalibrator(String filename)

    {
    	// Calibration variables
        calibFilename = filename;
        try {
			KUtils.loadCalibFromConfig(new ConfigFile(filename));
		} catch (IOException e) {
			e.printStackTrace();
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

        frame.add(visCanvas, BorderLayout.CENTER);


        // Set up labels to choose from and buttons for skipping/commiting
        ParameterGUI paramGUI = new ParameterGUI();
        paramGUI.addButtons("ido", "Choose Origin", "idx", "Choose X Direction", "idy", "Choose Y Direction", "test", "Test Point", "createCalib", "Create Calibration File", "new", "New Frame");
        paramGUI.addListener(new ParameterListener() {
            public void parameterChanged(ParameterGUI pg, String name) {
                if (name.equals("createCalib")) {
                    createCalibFile(calibFilename);
                } else if(name.equals("ido")){
                	curMode = Mode.ORIGIN;
                } else if(name.equals("idx")){
                	curMode = Mode.X;
                } else if(name.equals("idy")){
                	curMode = Mode.Y;
                } else if(name.equals("test")){
                	curMode = Mode.TEST;
                } else if(name.equals("new")){
                	ks = null;
                }
            }
        });
        frame.add(paramGUI, BorderLayout.SOUTH);



        // Finalize JFrame
        frame.setSize(FRAME_WIDTH, FRAME_HEIGHT);
        frame.setVisible(true);


        myLCM = LCM.getSingleton();
        myLCM.subscribe("KINECT_STATUS", this);
    }

    public void messageReceived(LCM lcm, String channel, LCMDataInputStream ins)
    {
    	if(ks != null){
    		return;
    	}
        try {
            ks = new kinect_status_t(ins);
            redrawImage();
        }catch (IOException e){
        	ks = null;
            e.printStackTrace();
            return;
        }
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

    		location[0] = intersect[0];
    		location[1] = intersect[1];
    		location[2] = 1; // In front of the image for the buffer

    		if(curMode == Mode.ORIGIN){
    	    	System.out.println("---- ORIGIN ----");
    			originLocation = location;
    		} else if(curMode == Mode.X){
    	    	System.out.println("---- X Point ----");
    			xLocation = location;
    		} else if(curMode == Mode.Y){
    			System.out.println("---- Y Point ---");
    			yLocation = location;
    		} else {
    			testLocation = location;
    		}

    		//System.out.println(v2s(getKinectPoint((int)location[0], (int)location[1])));


    		redrawImage();


            return false;
        }
    }

    public static String v2s(double[] v){
    	return String.format("(%f, %f, %f)", v[0], v[1], v[2]);
    }

    /** Writes the kinect calibration information into the file **/
    public void createCalibFile(String filename){

    	if(xLocation == null || originLocation == null || yLocation == null){
    		System.out.println("!!! kinect.calib not saved - needs the origin, x, and y specified !!!");
    		return;
    	}
    	// Find the ends of the vector to be treated as x
    	double[] originPoint = getKinectPoint((int)originLocation[0], (int)originLocation[1]);
    	double[] xPoint = getKinectPoint((int)xLocation[0], (int)xLocation[1]);
    	double[] yPoint = getKinectPoint((int)yLocation[0], (int)yLocation[1]);

    	System.out.println("O: " + v2s(originPoint));
    	System.out.println("X: " + v2s(xPoint));
    	System.out.println("Y: " + v2s(yPoint));


    	// Find the world Matrix in kinect coordinates
    	double[] worldX = LinAlg.normalize(LinAlg.subtract(xPoint, originPoint));
    	double[] worldY = LinAlg.normalize(LinAlg.subtract(yPoint,  originPoint));
    	double[] worldZ = LinAlg.crossProduct(worldX, worldY);
    	worldY = LinAlg.crossProduct(worldZ, worldX);


    	System.out.println("O: " + v2s(worldX));
    	System.out.println("X: " + v2s(worldY));
    	System.out.println("Y: " + v2s(worldZ));

    	// Translates kinect coordinates to the origin of the world coordinate system
    	double[][] k2wTranslate = new double[][]{
    			{1, 0, 0, 0},
    			{0, 1, 0, 0},
    			{0, 0, 1, 0},
    			{-originPoint[0], -originPoint[1], -originPoint[2], 1}
    	};
    	// Transform from world basis to kinect basis
    	double[][] w2kTransform = new double[][]{
    			{worldX[0], worldX[1], worldX[2], 0},
    			{worldY[0], worldY[1], worldY[2], 0},
    			{worldZ[0], worldZ[1], worldZ[2], 0},
    			{0, 0, 0, 1}
    	};
    	// Translates in world coordinates from the base to the origin of the arm
    	double[][] wTranslate = new double[][]{
    			{1, 0, 0, 0},
    			{0, 1, 0, 0},
    			{0, 0, 1, 0},
                {x_offset, 0, 0, 1}
    	};

    	// Overall transform is k2wTranslate * inv(w2kTransform) * wTranslate
    	// (translate to the world origin, then transform to the world basis, then translate within the world)
    	double[][] k2wTransform = LinAlg.matrixAB(LinAlg.matrixAB(k2wTranslate, LinAlg.inverse(w2kTransform)), wTranslate);
    	KUtils.kinectToWorldXForm = k2wTransform;

    	// Write the file out
    	try{
    		BufferedWriter out = new BufferedWriter(new FileWriter(calibFilename));
    		System.out.println("Writing transformation matrix");
            out.write("calibration {\n");
            out.write("\txform = [ ");
    		for(int i = 0; i < 4; i++){
    			for(int j = 0; j < 4; j++) {
                    if (i == 3 && j == 3) {
                        out.write(k2wTransform[i][j] + " ];\n");
                    } else {
                        out.write(k2wTransform[i][j] + ", ");
                    }
    			}
                System.out.println(String.format("|%5f, %5f, %5f, %5f|",
                                                 k2wTransform[i][0],
                                                 k2wTransform[i][1],
                                                 k2wTransform[i][2],
                                                 k2wTransform[i][3]));
    		}
    		out.write("\tborders = ");
    		out.write(String.format("[%d, %d, %d, %d];\n", (int)KUtils.viewRegion.getMinX(),
    				(int)KUtils.viewRegion.getMinY(), (int)KUtils.viewRegion.getMaxX(),
    				(int)KUtils.viewRegion.getMaxY()));
            out.write("}\n");
    		out.close();
    	} catch (IOException e){
    		System.out.println("Couldn't write kinect.calib");
    	}

    	// Tests, should come out to unit vectors
    	/*
    	double[] a = KUtils.getWorldCoordinates(LinAlg.add(worldX, originPoint));
    	System.out.println(String.format("(%f, %f, %f)", a[0], a[1], a[2]));
    	a = KUtils.getWorldCoordinates(LinAlg.add(worldY, originPoint));
    	System.out.println(String.format("(%f, %f, %f)", a[0], a[1], a[2]));
    	a = KUtils.getWorldCoordinates(LinAlg.add(worldZ, originPoint));
    	System.out.println(String.format("(%f, %f, %f)", a[0], a[1], a[2]));
    	*/
    }

    /** Takes a point in image coordinates and returns a point in the space of the kinect **/
    public double[] getKinectPoint(int x, int y){
    	y = KINECT_HEIGHT-y;
		int index = y*KINECT_WIDTH + x;
		int depth = ((ks.depth[2*index+1]&0xff) << 8) | (ks.depth[2*index+0]&0xff);
		double[] xyzrgb = KUtils.getRegisteredXYZRGB(x, y, ks);
    	//System.out.println("Point: " + x + ", " + y);
    	//System.out.println("Depth: " + depthLookUp[depth]);
    	//System.out.println(String.format("(%f, %f, %f)", xyzrgb[0], xyzrgb[1], xyzrgb[2]));
    	return new double[]{xyzrgb[0], xyzrgb[1], xyzrgb[2]};
    }

    /** Draws the image on the screen with circles where the user has clicked **/
    public void redrawImage(){
    	BufferedImage image = new BufferedImage(KINECT_WIDTH, KINECT_HEIGHT, BufferedImage.TYPE_3BYTE_BGR);
        byte[] buf = ((DataBufferByte)(image.getRaster().getDataBuffer())).getData();
        for (int i = 0; i < buf.length; i+=3) {
            buf[i] = ks.rgb[i+2];	// B
            buf[i+1] = ks.rgb[i+1];	// G
            buf[i+2] = ks.rgb[i];	// R
        }

        // Render the kinect image
        VisWorld.Buffer visBuffer = visWorld.getBuffer("backgroundImage");
        visBuffer.addBack(new VzImage(image, VzImage.FLIP));

        // Draw a circle at the base and head locations
        if(originLocation != null){
            VzCircle circle = new VzCircle(4, new VzLines.Style(Color.red, 2));
            VisChain vch = new VisChain(LinAlg.translate(originLocation), circle);

            visBuffer.addBack(vch);
        }
        if(xLocation != null){
            VzCircle circle = new VzCircle(4, new VzLines.Style(Color.green, 2));
            VisChain vch = new VisChain(LinAlg.translate(xLocation), circle);
            visBuffer.addBack(vch);
        }
        if(yLocation != null){
            VzCircle circle = new VzCircle(4, new VzLines.Style(Color.yellow, 2));
            VisChain vch = new VisChain(LinAlg.translate(yLocation), circle);
            visBuffer.addBack(vch);
        }
        if(testLocation != null){
            VzCircle circle = new VzCircle(4, new VzLines.Style(Color.blue, 2));
            VisChain vch = new VisChain(LinAlg.translate(testLocation), circle);
            visBuffer.addBack(vch);

        	double[] testPt = getKinectPoint((int)testLocation[0], (int)testLocation[1]);
        	double[] a = KUtils.getWorldCoordinates(testPt);
        	//System.out.println(String.format("(%f, %f, %f)", testPt[0], testPt[1], testPt[2]));
        	System.out.println(String.format("(%f, %f, %f)", a[0], a[1], a[2]));
        }

        VzRectangle rect = new VzRectangle(KUtils.viewRegion.getWidth(), KUtils.viewRegion.getHeight(), new VzLines.Style(Color.white, 2));
        double[] t = new double[]{KUtils.viewRegion.getX() + KUtils.viewRegion.getWidth()/2,
        		KINECT_HEIGHT - (KUtils.viewRegion.getY() + KUtils.viewRegion.getHeight()/2), .01};
        VisChain rectVC = new VisChain(LinAlg.translate(t), rect);
        visBuffer.addBack(rectVC);

        visBuffer.swap();
    }


    public static void main(String[] args)
    {
    	GetOpt opts = new GetOpt();

        opts.addBoolean('h', "help", false, "Show this help screen");
        opts.addString('c', "config", null, "Configuration file to write to");
        opts.addString('a', "arm-config", null, "Configuration file designating any arm parameters necessary for calibration");

        if (!opts.parse(args) || opts.getBoolean("help") || opts.getExtraArgs().size() > 0) {
            opts.doHelp();
            return;
        }

        if (opts.getString("config") == null) {
            System.out.println("Usage: Must specify a configuration file");
            opts.doHelp();
            return;
        }

        if (opts.getString("arm-config") == null) {
            System.err.println("Usage: Must specify an arm configuration file");
            opts.doHelp();
            return;
        }

        Config armConfig;
        try {
            armConfig = new ConfigFile(opts.getString("arm-config"));
        } catch (IOException ioex) {
            System.err.println("ERR: Error opening arm configuration file");
            ioex.printStackTrace();
            return;
        }
        String name = armConfig.getString("arm.arm_version", null);
        assert (name != null);
        x_offset = armConfig.getDouble("arm."+name+".calib_offset", 0);


        //segment = new Segment(da, true);
        KinectCalibrator kc = new KinectCalibrator(opts.getString("config"));
    }
}
