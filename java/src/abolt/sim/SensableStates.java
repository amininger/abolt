package abolt.sim;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;

public class SensableStates {
	private HashMap<String, ArrayList<String>> possibleStates;
	private HashMap<String, String> currentState;
	
	public SensableStates(){
		possibleStates = new HashMap<String, ArrayList<String>>();
		currentState = new HashMap<String, String>();
	}
	
	public void addStateSet(String stateName, ArrayList<String> allowedStates){
		if(allowedStates.size() == 0){
			return;
		}
		possibleStates.put(stateName, allowedStates);
		currentState.put(stateName, allowedStates.get(0));
	}
	
	public String getState(String stateName){
		return currentState.get(stateName);
	}
	
	public void setState(String stateName, String newState){
		if(!possibleStates.containsKey(stateName)){
			return;
		}
		if(possibleStates.get(stateName).contains(newState)){
			currentState.put(stateName, newState);
		}
	}
	
	public void setState(String keyValString){
		String[] keyValPair = keyValString.split("=");
		if(keyValPair.length < 2){
			return;
		}
		setState(keyValPair[0], keyValPair[1]);
	}
	
	public String getProperties(){
		StringBuilder properties = new StringBuilder();
		for(Map.Entry<String, String> state : currentState.entrySet()){
			properties.append(String.format("%s=%s,", state.getKey(), state.getValue()));
		}
		return properties.substring(0, properties.length() - 1);
	}
	
	public ArrayList<String> getAllowedStates(String stateName){
		return possibleStates.get(stateName);
	}
}
