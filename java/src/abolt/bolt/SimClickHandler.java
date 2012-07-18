package abolt.bolt;

import abolt.kinect.ObjectInfo;
import abolt.objects.BoltObject;
import abolt.sim.SimSensable;
import abolt.util.SimUtil;
import april.sim.SimObject;

public interface SimClickHandler {
	/** 
	 * clicked(obj) - called when an object in the simulator is clicked
	 * If you want to change which object is currently selected return it's id, 
	 * Otherwise (to leave things unchanged) return -1
	 */
	int clicked(BoltObject obj);
	int clicked(SimObject obj);
	
	public class SelectObject implements SimClickHandler{
		public int clicked(BoltObject obj){
			return obj.getID();
		}
		public int clicked(SimObject obj){
			if(obj instanceof SimSensable){
				return ((SimSensable)obj).getID();
			} else {
				return -1;
			}	
		}
	}
	
	public class ChangeId implements SimClickHandler{
		public int clicked(BoltObject obj){
			ObjectInfo info = obj.getInfo();
			if(info.createdFrom != null){
				int id = SimUtil.nextID();
				info.createdFrom.setID(id);
				return id;
			}
			return -1;
		}
		public int clicked(SimObject obj){
			return -1;
		}
	}
	
	public class ToggleVisibility implements SimClickHandler{
		public int clicked(BoltObject obj){
			obj.setVisible(!obj.isVisible());
			return -1;
		}
		public int clicked(SimObject obj){
			return -1;
		}
	}
}
