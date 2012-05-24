package abolt.objects;

import java.util.ArrayList;
import java.util.HashMap;

import abolt.lcmtypes.object_data_t;
import abolt.sim.SimActionable;
import abolt.sim.SimSensable;

public class SensableManager {
	private HashMap<Integer, SimSensable> sensables;
	
	public SensableManager(){
		sensables = new HashMap<Integer, SimSensable>();
	}
	
	public HashMap<Integer, SimSensable> getSensables(){
		return sensables;
	}
	
	public void addSensable(SimSensable sensable){
		synchronized(sensables){
			sensables.put(sensable.getID(), sensable);
		}
	}
	
	public String[] getSensableStrings(){
		ArrayList<String> sensableStrings = new ArrayList<String>();
		synchronized(sensables){
			for(SimSensable sens : sensables.values()){
				String senString = sens.getProperties();
				if(senString != null){
					sensableStrings.add(senString);
				}
			}
		}
		return sensableStrings.toArray(new String[0]);
	}
	
	public void performAction(String action){
		// Expecting the form 'ID=234,DOOR=OPEN'
		String[] args = action.toUpperCase().split(",");
		if(args.length < 2){
			return;
		}
		String[] idArg = args[0].split("=");
		if(idArg.length < 2 || !idArg[0].equals("ID")){
			return;
		}
		int id = Integer.parseInt(idArg[1]);
		synchronized(sensables){
			SimSensable sens = sensables.get(id);
			if(sens == null || !(sens instanceof SimActionable)){
				return;
			}
			((SimActionable)sens).setState(args[1]);
		}
	}
}
