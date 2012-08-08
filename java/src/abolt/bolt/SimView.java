package abolt.bolt;

import java.util.ArrayList;

import abolt.collision.ShapeToVisObject;
import abolt.objects.*;
import april.vis.*;
import april.vis.VisWorld.Buffer;

public interface SimView {
	void draw(VisWorld.Buffer buffer);
	
	public class PointCloudView implements SimView{
		public void draw(Buffer buffer) {
			BoltObjectManager objManager = BoltObjectManager.getSingleton();
	    	synchronized(objManager.objects){
				for(BoltObject obj : objManager.objects.values()){
					ArrayList<double[]> points = obj.getInfo().points;
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
				}
	    	}
		}
	}
	
	public class SoarView implements SimView{
		public void draw(VisWorld.Buffer buffer){
			BoltObjectManager objManager = BoltObjectManager.getSingleton();
	    	synchronized(objManager.objects){
				for(BoltObject obj : objManager.objects.values()){
					if(obj.getInfo().createdFrom != null){
						ISimBoltObject simObj = obj.getInfo().createdFrom;
						ArrayList<VisObject> visObjs = ShapeToVisObject.getVisObjects(simObj.getAboltShape(), new VzMesh.Style(simObj.getColor()));
						for(VisObject visObj : visObjs){
	    					buffer.addBack(new VisChain(simObj.getPose(), visObj));
						}
					} else {
		    			buffer.addBack(obj.getVisObject());
					}
		    	}
	    	}	
		}
	}
}
