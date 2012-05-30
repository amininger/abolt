package abolt.bolt;

import java.util.ArrayList;
import java.util.HashMap;

import abolt.objects.BoltObject;
import april.sim.SimObject;
import april.vis.VisCanvas;
import april.vis.VisLayer;
import april.vis.VisObject;
import april.vis.VisWorld;

public interface IBoltGUI {
	 void drawObjects(HashMap<Integer, BoltObject> objects);
	 void drawVisObjects(String bufferName, ArrayList<VisObject> objects);
	 VisCanvas getCanvas();
	 VisLayer getLayer();
	 BoltObject getSelectedObject();
}
