package abolt.bolt;

import java.awt.Color;
import java.util.ArrayList;
import java.util.HashMap;

import abolt.classify.ColorFeatureExtractor;
import abolt.collision.ShapeToVisObject;
import abolt.sim.SimBoltObject;
import april.jmat.LinAlg;
import april.vis.*;
import april.vis.VisWorld.Buffer;

public interface SimView {
	void draw(VisWorld.Buffer buffer);
	
	public class PointCloudView implements SimView{
		public void draw(Buffer buffer) {
			HashMap<Integer, BoltObject> objects = Perception.getSingleton().getCurrentObjects();
			for(BoltObject obj : objects.values()){
				ArrayList<double[]> points = obj.getPoints();
				if(points != null && points.size() > 0){
	    			VisColorData colors = new VisColorData();
	    			VisVertexData vertexData = new VisVertexData();
	    			for(int i = 0; i < points.size(); i++){
	    				double[] pt = points.get(i);
	    				vertexData.add(new double[]{pt[0], pt[1], pt[2]});
		    			colors.add((int)points.get(i)[3]);
	    			}
	    			VzPoints visPts = new VzPoints(vertexData, new VzPoints.Style(colors, 2));
	    			buffer.addBack(new VisLighting(false, LinAlg.translate(0, 0, 0), visPts));
				}
			}
		}
	}
	
	public class SoarView implements SimView{
		public void draw(VisWorld.Buffer buffer){
			HashMap<Integer, BoltObject> objects = Perception.getSingleton().getCurrentObjects();
			for(BoltObject obj : objects.values()){
				if(obj.sourceObject != null){
					SimBoltObject simObj = obj.sourceObject;
					ArrayList<VisObject> visObjs = ShapeToVisObject.getVisObjects(simObj.getAboltShape(), new VzMesh.Style(simObj.getColor()));
					for(VisObject visObj : visObjs){
    					buffer.addBack(new VisChain(simObj.getPose(), visObj));
					}
				} else {
					Color color = ColorFeatureExtractor.getColorFromFeatures(ColorFeatureExtractor.getFeatures(obj));
					double[][] bbox = obj.getBBox();
			        VisObject model = new VisChain(LinAlg.translate(obj.getPos()), 
			        		LinAlg.scale(bbox[1][0] - bbox[0][0], bbox[1][1] - bbox[0][1], bbox[1][2] - bbox[0][2]),
			                new VzBox(new VzMesh.Style(color)));
	    			buffer.addBack(model);
				}
	    	}
		}
	}
}
