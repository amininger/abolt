package abolt.bolt;

import java.util.HashMap;

import abolt.objects.BoltObject;
import april.sim.SimObject;
import april.vis.VisCanvas;
import april.vis.VisWorld;

public interface IBoltGUI {
	 void drawObjects(HashMap<Integer, BoltObject> objects);
	 VisCanvas getCanvas();
	 BoltObject getSelectedObject();
}
