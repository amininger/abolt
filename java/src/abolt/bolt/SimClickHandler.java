package abolt.bolt;

import abolt.util.SimUtil;

public interface SimClickHandler {
	/** 
	 * clicked(obj) - called when an object in the simulator is clicked
	 * If you want to change which object is currently selected return it's id, 
	 * Otherwise (to leave things unchanged) return -1
	 */
	int clicked(BoltObject obj);
	
	public class SelectObject implements SimClickHandler{
		public int clicked(BoltObject obj){
			return obj.getID();
		}
	}
	
	public class ChangeId implements SimClickHandler{
		public int clicked(BoltObject obj){
			if(obj.sourceObject != null){
				obj.sourceObject.setID(SimUtil.nextID());
				return obj.sourceObject.getID();
			}
			return -1;
		}
	}
	
	public class ToggleVisibility implements SimClickHandler{
		public int clicked(BoltObject obj){
			if(obj.sourceObject != null){
				obj.sourceObject.setVisible(!obj.sourceObject.getVisible());
			}
			return -1;
		}
	}
}
