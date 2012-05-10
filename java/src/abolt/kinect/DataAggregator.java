package abolt.kinect;

import april.util.*;
import lcm.lcm.*;
import java.util.*;

public class DataAggregator{
	public int WIDTH = 640;
	public int HEIGHT = 480;
    final static int MAX_HISTORY = 100;

	public float[] depthLookUp = new float[2048];          //holds depth conversions so we only have to calculate them once
    public double[] t;

	public ArrayList<double[]> currentPoints;                //array of all depth points mapped to xyz grid
	public ArrayList<double[]> coloredPoints;
	public HashMap<Integer, ObjectInfo> objects;           //map of all objects found in current frame to their data
	public HashMap<Integer, ObjectInfo> prevObjects;       //map of all objects found in previous frame
	public HashMap<Integer, Integer> map;                  //map of object ID to color
	public HashMap<Integer, Integer> prevMap;              // "                    "   for previous frame
    public ArrayList<HashMap<Integer,ObjectInfo>> history;

	public UnionFindSimple ufs;                            //Union Find class for keeping track of union find algo

    public double colorThresh, unionThresh, ransacPercent, ransacThresh;

    public DataAggregator(boolean record){
        t = new double[] { -0.0254, -0.00013, -0.00218 }; //-2.8831221109805402e-02, -1.6603848659438847e-06, -6.5479395637054133e-04
        objects = new HashMap<Integer, ObjectInfo>();
        prevObjects = new HashMap<Integer, ObjectInfo>();
        map = new HashMap<Integer, Integer>();
        prevMap = new HashMap<Integer, Integer>();
        history = new ArrayList<HashMap<Integer,ObjectInfo>>();
    }


    public void newFrame()
    {
        if(history.size() > 0 && objects.size() > prevObjects.size()){

        	boolean foundMatches = false;
        	int step = 2;

        	// Debugging simplification
        	ArrayList<Integer> noMatch = new ArrayList<Integer>();
        	HashMap<Integer, Integer> alreadyAssigned = new HashMap<Integer,Integer>();
        	Set<Integer> setN = objects.keySet();
        	for(Integer i : setN){
        		if (objects.get(i).matched == true)
        			alreadyAssigned.put(i,i);
        		else noMatch.add(i);
        	}

        	//System.out.println("GAINED "+noMatch.size()+" OBJECTS");
        	while(!foundMatches && step < history.size()){
        		HashMap<Integer,ObjectInfo> old = history.get(history.size()-step);
        		boolean[] matched = new boolean[noMatch.size()];

        		// For each unmatched object, try to find a similar object
        		// from this scene
        		for(int i=0; i<noMatch.size(); i++){
        			ObjectInfo oi = objects.get(noMatch.get(i));
        			int mostSim = oi.mostSimilar(old, alreadyAssigned);

        			// Give object ID and coloring of most similar object
        			if (mostSim >= 0 && !alreadyAssigned.containsKey(i)){
        				int newID = old.get(mostSim).repID;
        				int newColor = old.get(mostSim).color;
        				oi.equateObject(newID, newColor);
        				oi.matched = true;
        				alreadyAssigned.put(newID, newID);
        				for(double[] p : oi.points){
        					coloredPoints.add(new double[]{p[0], p[1], p[2], oi.color});
        				}
        				matched[i] = true;

        			}
        		}
        		// Remove ones that were matched
        		for(int i=matched.length-1; i>-1; i--)
        			if(matched[i]) noMatch.remove(i);
        		foundMatches = (noMatch.size() == 0);
        		step ++;
        	}
        }

	    // Add new frame to history (create new HashMap)
	    HashMap<Integer, ObjectInfo> forHistory = new HashMap<Integer, ObjectInfo>();
	    Set<Integer> set = objects.keySet();
	    for(Integer i : set)
	        forHistory.put(i, objects.get(i));
	    history.add(forHistory);
	    // If history is too long, remove the oldest
	    if(history.size() > MAX_HISTORY){
	        history.remove(0);
	    }

    }
}
