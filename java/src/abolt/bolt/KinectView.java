package abolt.bolt;

import java.awt.BorderLayout;
import java.awt.Color;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Timer;
import java.util.TimerTask;

import javax.swing.JFrame;

import lcm.lcm.LCM;
import lcm.lcm.LCMDataInputStream;
import lcm.lcm.LCMSubscriber;

import abolt.kinect.KinectCamera;
import abolt.lcmtypes.kinect_status_t;
import abolt.objects.BoltObject;
import abolt.objects.BoltObjectManager;
import april.jmat.LinAlg;
import april.vis.*;

public class KinectView extends JFrame implements LCMSubscriber{
    final int KINECT_WIDTH = kinect_status_t.WIDTH;
    final int KINECT_HEIGHT = kinect_status_t.HEIGHT;

    VisWorld visWorld;
    VisLayer visLayer;

    // The most recently accessed kinect status
    kinect_status_t ks = null;
    
    private Timer updateTimer;
    private static final int UPDATE_RATE = 10; // # updates per second

    public KinectView(){

        // Setup Frame
        JFrame frame = new JFrame("Kinect View");
        frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        frame.setLayout(new BorderLayout());

        // Initialize the image frame and canvas
        visWorld = new VisWorld();

        visLayer = new VisLayer(visWorld);
        VisCanvas visCanvas = new VisCanvas(visLayer);

        //Set up initial camera view
        visLayer.cameraManager.uiLookAt(new double[] {1, 0, .7},// Camera position
                                  new double[] {0, 0, 0},// Point looking at
                                  new double[] {0, 0, 1},// Up
                                  false);


        frame.add(visCanvas, BorderLayout.CENTER);

        // Finalize JFrame
        frame.setSize(400, 300);
        frame.setVisible(true);
        
		class RefreshTask extends TimerTask{
			public void run() {
				update();
			}
    	}
		updateTimer = new Timer();
		updateTimer.schedule(new RefreshTask(), 1000, 1000/UPDATE_RATE);

        LCM lcm = LCM.getSingleton();
        lcm.subscribe("KINECT_STATUS", this);
    }

    public void messageReceived(LCM lcm, String channel, LCMDataInputStream ins){
		try {
            ks = new kinect_status_t(ins);
            redrawKinectData();
        }catch (IOException e){
        	ks = null;
            e.printStackTrace();
            return;
        }
	}
    
    private void update(){
    	VisWorld.Buffer buffer = visWorld.getBuffer("objects");
		BoltObjectManager objManager = BoltObjectManager.getSingleton();
    	synchronized(objManager.objects){
    		for(BoltObject obj : objManager.objects.values()){
    			double[][] bbox = obj.getBBox();
    			double[][] scale = LinAlg.scale(bbox[1][0] - bbox[0][0], bbox[1][1] - bbox[0][1], bbox[1][2] - bbox[0][2]);
    			VzBox box = new VzBox(new VzMesh.Style(new Color(0,0,0,0)), new VzLines.Style(Color.cyan, 2));
    			buffer.addBack(new VisChain(LinAlg.translate(obj.getPose()), scale, box));
    		}
    	}
		buffer.swap();
    }
    
    private void redrawKinectData(){
    	VisWorld.Buffer buffer = visWorld.getBuffer("kinect");
    	ArrayList<double[]> points = KinectCamera.extractPointCloudData(ks);
		if(points != null && points.size() > 0){
			VisColorData colors = new VisColorData();
			VisVertexData vertexData = new VisVertexData();
			for(int i = 0; i < points.size(); i++){
				double[] pt = Bolt.getCamera().getWorldCoords(points.get(i));
				vertexData.add(new double[]{pt[0], pt[1], pt[2]});
    			colors.add((int)points.get(i)[3]);
			}
			VzPoints visPts = new VzPoints(vertexData, new VzPoints.Style(colors, 2));
			buffer.addBack(visPts);
		}
		buffer.swap();
    }
}
