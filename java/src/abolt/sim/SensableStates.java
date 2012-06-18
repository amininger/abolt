package abolt.sim;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;

public class SensableStates {
	private HashMap<String, String[]> possibleStates;
	private HashMap<String, String> currentState;
	
	public SensableStates(){
		possibleStates = new HashMap<String, String[]>();
		currentState = new HashMap<String, String>();
	}
	
	public SensableStates(String[] props){
		possibleStates = new HashMap<String, String[]>();
		currentState = new HashMap<String, String>();
		for(String prop : props){
			String[] nameVals = prop.split("=");
			if(nameVals.length >= 2){
				addStateSet(nameVals[0], nameVals[1].split(","));
			}
		}
	}
	
	public void addStateSet(String stateName, String[] allowedStates){
		if(allowedStates.length == 0){
			return;
		}
		for(int i = 0; i < allowedStates.length; i++){
			allowedStates[i] = allowedStates[i].toLowerCase();
		}
		possibleStates.put(stateName.toLowerCase(), allowedStates);
		currentState.put(stateName.toLowerCase(), allowedStates[0]);
	}
	
	public String getState(String stateName){
		return currentState.get(stateName.toLowerCase());
	}
	
	public void setState(String stateName, String newState){
		String[] states = possibleStates.get(stateName.toLowerCase());
		if(states == null){
			return;
		}
		newState = newState.toLowerCase();
		for(int i = 0; i < states.length; i++){
			if(states[i].equals(newState)){
				currentState.put(stateName.toLowerCase(), newState);
				break;
			}
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
		if(currentState.size() == 0){
			return "";
		}
		StringBuilder properties = new StringBuilder();
		for(Map.Entry<String, String> state : currentState.entrySet()){
			properties.append(String.format("%s=%s,", state.getKey(), state.getValue()));
		}
		return properties.substring(0, properties.length() - 1) + ",";
	}
	
	public String[] getAllowedStates(String stateName){
		return possibleStates.get(stateName.toLowerCase());
	}
}
