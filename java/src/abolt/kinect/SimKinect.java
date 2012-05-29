package abolt.kinect;

import java.util.ArrayList;
import java.util.Date;
import java.util.HashMap;
import java.util.Timer;
import java.util.TimerTask;


import abolt.bolt.Bolt;
import abolt.bolt.BoltSimulator;
import abolt.classify.ClassifierManager;
import abolt.collision.*;
import abolt.objects.BoltObject;
import abolt.objects.BoltObjectManager;
import abolt.objects.IObjectManager;
import abolt.objects.ISimBoltObject;
import abolt.objects.SimBoltObject;
import abolt.objects.SimObjectManager;
import april.jmat.LinAlg;
import april.sim.SimObject;
import april.vis.VisCameraManager.CameraPosition;

public class SimKinect implements IBoltCamera{
	private static final double PIXEL_SIZE = .001;
	private static final double DIST_TO_SCREEN = .2;
	private static final double[] up = new double[]{0, 1, 0};
	
	private int width;
	private int height;
	
	private double[] origin;
	// Using a left handed coordinate system
	// From the camera's perspective, X is to the right, Y is up
	//    and Z is forward along the view direction
	private double[] xAxis;
	private double[] yAxis;
	private double[] zAxis;
	
	private double[] screenOrigin;
	
    private Timer updateTimer;
    private static final int UPDATE_RATE = 2; // # updates per second
	
	public SimKinect(int width, int height){
		this.width = width;
		this.height = height;
		
		origin = new double[]{0, 0, 0};
		zAxis = new double[]{0, 0, -1};
		
		class RefreshTask extends TimerTask{
			public void run() {
				update();
			}
    	}
		updateTimer = new Timer();
		updateTimer.schedule(new RefreshTask(), 1000, 1000/UPDATE_RATE);
	}
	
	private void update(){
		long time = new Date().getTime();
		
		updatePosition(Bolt.getBoltGUI().getLayer().cameraManager.getCameraTarget());
		HashMap<Integer, ObjectInfo> info = new HashMap<Integer, ObjectInfo>();
		if(!(Bolt.getBoltGUI() instanceof BoltSimulator)){
			System.err.println("SimKinect update: Expecting BoltSimulator as the GUI");
			return;
		}
		BoltSimulator sim = (BoltSimulator)Bolt.getBoltGUI();
		synchronized(sim.getWorld().objects){
			for(SimObject obj : sim.getWorld().objects){
				if(!(obj instanceof ISimBoltObject)){
					continue;
				}
				ObjectInfo objInfo = constructObjectInfo((ISimBoltObject)obj);
				if(objInfo != null){
					info.put(objInfo.repID, objInfo);
				}
			}
		}
		Bolt.getObjectManager().updateObjects(info);
		long dif = new Date().getTime() - time;
		System.out.println(dif);
	}
	
	private void updatePosition(CameraPosition camPos){
		origin = camPos.eye.clone();
		zAxis = LinAlg.subtract(camPos.lookat, camPos.eye);
		LinAlg.normalizeEquals(zAxis);
		
		calcCoordinateSystem();
	}
	
	private void calcCoordinateSystem(){
		xAxis = LinAlg.crossProduct(zAxis, up);
		yAxis = LinAlg.crossProduct(xAxis, zAxis);
		LinAlg.normalizeEquals(xAxis);
		LinAlg.normalizeEquals(yAxis);
		
		screenOrigin = LinAlg.add(origin, LinAlg.scale(zAxis, DIST_TO_SCREEN));
	}
	
	private ObjectInfo constructObjectInfo(ISimBoltObject obj){
		if(!(obj instanceof ISimBoltObject)){
			return null;
		}
		Shape shape = ((ISimBoltObject)obj).getAboltShape();
		int color = ((ISimBoltObject)obj).getColor().getRGB();
		double[][] T = obj.getPose();
		double[] pose = LinAlg.matrixToXyzrpy(T);
		
		int[] pixel = getPixel(pose);
		if(pixel == null){
			return null;
		}

		ArrayList<double[]> points = new ArrayList<double[]>();
		if(!collideShape(points, pixel[0], pixel[1], shape, T)){
			// For some reason there was no collision with the center of the shape, something is odd
			return null;
		}
		
		ObjectInfo info = new ObjectInfo(color, obj.getID(), points.get(0));
		info.repID = obj.getID();
		
		int START_SIZE = 20;
		int left = pixel[0] - START_SIZE/2;
		if(left < -width/2){
			left = -width/2;
		}
		int right = pixel[0] + START_SIZE/2;
		if(right > width/2){
			right = width/2;
		}
		int top = pixel[1]+ START_SIZE/2;
		if(top > height/2){
			top = height/2;
		}
		int bot = pixel[1] - START_SIZE/2;
		if(bot < -height/2){
			bot = -height/2;
		}
		
		for(int x = left; x <= right; x++){
			for(int y = top; y >= bot; y--){
				collideShape(points, x, y, shape, T);
			}
		}
		
		// Keep expanding the borders until out of the view region or don't hit any more points
		boolean contL = true, contR = true, contT = true, contB = true;	//continue left, right, top, bot
		while(contL || contR || contT || contB){
			if(contL){
				left--;
				contL = scanVertical(points, left, top, bot, shape, T);
			}
			if(contR){
				right++;
				contR = scanVertical(points, right, top, bot, shape, T);
			}
			if(contT){
				top++;
				contT = scanHorizontal(points, top, left, right, shape, T);
			}
			if(contB){
				bot--;
				contB = scanHorizontal(points, bot, left, right, shape, T);
			}		
		}
		
		for(double[] pt : points){
			pt[3] = color;
			info.update(pt);
		}
		
		return info;
	}
	
	// Scan a horizontal row of pixels from left to right inclusive and add collisions to the points array
	// Returns true if any collisions occurred (returns false if y is out of bounds automatically)
	private boolean scanHorizontal(ArrayList<double[]> points, int y, int left, int right, Shape shape, double[][] T){
		if(y < -height/2 || y > height/2){
			return false;
		}
		boolean hitPoint = false;
		for(int x = left; x <= right; x++){
			hitPoint = collideShape(points, x, y, shape, T) || hitPoint;
		}
		return hitPoint;
	}
	
	// Scan a vertical row of pixels from top to bottom inclusive and add collisions to the points array
	// Returns true if any collisions occurred (returns false if x is out of bounds automatically)
	private boolean scanVertical(ArrayList<double[]> points, int x, int top, int bot, Shape shape, double[][] T){
		if(x < -width/2 || x > width/2){
			return false;
		}
		boolean hitPoint = false;
		for(int y = top; y >= bot; y--){
			hitPoint = collideShape(points, x, y, shape, T) || hitPoint;
		}
		return hitPoint;
	}
	
	// Calculates a collision for the shape from the camera's origin through the given pixel
	// If one occurs, it adds the point to the points array and returns true
	private boolean collideShape(ArrayList<double[]> points, int x, int y, Shape shape, double[][] T){
		double[] pixel = LinAlg.add(screenOrigin, LinAlg.scale(xAxis, x * PIXEL_SIZE));
		pixel = LinAlg.add(pixel, LinAlg.scale(yAxis, y * PIXEL_SIZE));
		double[] dir = LinAlg.subtract(pixel, origin);
		LinAlg.normalizeEquals(dir);
		double d = shape.collisionRay(origin, dir, T);
		
		if(d == Double.MAX_VALUE){
			return false;
		}
		double[] pt = LinAlg.add(origin, LinAlg.scale(dir, d));
		points.add(new double[]{pt[0], pt[1], pt[2], 0});
		return true;
	}

	@Override
	public int[] getPixel(double[] cameraPt) {
		double[] pt = LinAlg.subtract(LinAlg.resize(cameraPt, 3), origin);
		// Project the relative position of the shape onto each axis
		double zproj = LinAlg.dotProduct(pt, zAxis);
		double xproj = LinAlg.dotProduct(pt, xAxis);
		double yproj = LinAlg.dotProduct(pt, yAxis);
		
		if(zproj <= 0){
			// The shape is behind the camera's view, don't consider it
			return null;
		}
		
		// Compute the projection of the relative position of the shape onto the screen
		double xPixel = xproj * DIST_TO_SCREEN / zproj / PIXEL_SIZE;
		double yPixel = yproj * DIST_TO_SCREEN / zproj / PIXEL_SIZE;
		if(Math.abs(xPixel) > width/2 || Math.abs(yPixel) > height/2){
			// The center of the shape is off screen, so we don't consider it
			return null;
		}
		return new int[]{(int)Math.round(xPixel), (int)Math.round(yPixel)};
	}

	@Override
	public double[] getWorldCoords(double[] cameraPt) {
		return cameraPt;
	}
}
