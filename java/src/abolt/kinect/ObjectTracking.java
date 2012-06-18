package abolt.kinect;

import java.util.*;

import april.jmat.*;
import april.util.*;
import abolt.classify.ColorFeatureExtractor;
import abolt.arm.*;

public class ObjectTracking
{
    final double MAX_TRAVEL_DIST = 0.05;//.1;
    final double MAX_COLOR_CHANGE = 60;//40;
    final int MAX_HISTORY = 10;
    private final static double darkThreshold = .4;

    private BoltArm arm;
    private HashMap<Integer, ObjectInfo> lastFrame;
    private HashMap<Integer, ObjectInfo> lostObjects;
    private HashMap<Integer, Long> lostTime;
    private ObjectInfo heldObject;

    enum ArmState
    {  GRABBING_OBJECT, HOLDING_OBJECT, DROPPING_OBJECT,
       JUST_DROPPED_OBJECT, WAITING,
    }
    private ArmState armState = ArmState.WAITING;

    public ObjectTracking()
    {
        arm = BoltArm.getSingleton();
        lastFrame = new HashMap<Integer, ObjectInfo>();
        lostTime = new HashMap<Integer, Long>();
        lostObjects = new HashMap<Integer, ObjectInfo>();
    }

    public void armGrabbing(Integer id)
    {
        if(armState == ArmState.WAITING
            || armState == ArmState.HOLDING_OBJECT){
            Integer hashID = -1;
            Set objects = lastFrame.keySet();
            for(Object obj : objects){
                if(lastFrame.get((Integer)obj).repID == id){
                    hashID = (Integer) obj;
                    break;
                }
            }

            heldObject = lastFrame.get(hashID);
            if(heldObject != null)
                armState = ArmState.GRABBING_OBJECT;
        }
    }

    public void armDropping(double[] finalLocation)
    {
        if(heldObject != null){
            armState = ArmState.DROPPING_OBJECT;

            heldObject.resetCenter(finalLocation);
            lastFrame.put(heldObject.repID, heldObject);
            lostObjects.remove(heldObject.ufsID);
        }
    }

    public void armWaiting()
    {
        if(armState == ArmState.GRABBING_OBJECT
           || armState == ArmState.HOLDING_OBJECT){
            armState = ArmState.HOLDING_OBJECT;
        }
        else if(armState == ArmState.DROPPING_OBJECT
                || armState == ArmState.JUST_DROPPED_OBJECT){
            armState = ArmState.JUST_DROPPED_OBJECT;
        }
        else
            armState = ArmState.WAITING;
    }

    public void armFailed()
    {
        heldObject = null;
        armState = ArmState.WAITING;
    }

    public double xydistance(double[] p1, double[] p2)
    {
        double dx = p1[0]-p2[0];
        double dy = p1[1]-p2[1];
        return Math.sqrt(dx*dx + dy*dy);
    }

    // Remove objects that are too dark
    public void removeDarkObjects(HashMap<Integer, ObjectInfo> currentFrame)
    {
        ArrayList<Integer> dark = new ArrayList<Integer>();
        for(ObjectInfo info : currentFrame.values()){
            ArrayList<Double> colorFeatures = ColorFeatureExtractor.getFeatures(info);
            if(colorFeatures.get(0) <= darkThreshold
               && colorFeatures.get(1) <= darkThreshold
               && colorFeatures.get(2) <= darkThreshold){
                dark.add(info.ufsID);
            }
        }
        for(Integer i : dark)
            currentFrame.remove(i);
    }


    /** Remove items in history that have been there too long and add newly
     *  lost objects to the history of lost objects. **/
    public void updateHistory(HashMap<Integer, Integer> unusedOld)
    {
        long currentTime = TimeUtil.utime();

        // Increment how long each of the lost objects has been around and
        // discard ones that are too old.
        Set lost = lostObjects.keySet();
        ArrayList<Integer> toRemove = new ArrayList<Integer>();
        for(Object lostObj : lost){
            double time  = (currentTime -lostTime.get((Integer) lostObj))/1000000;
            if(time >= MAX_HISTORY)
                toRemove.add((Integer) lostObj);
        }
        for(Integer id : toRemove){
            lostObjects.remove(id);
            lostTime.remove(id);
        }

        // Add lost objects to history
        Set old = unusedOld.keySet();
        for(Object id : old){
            lostObjects.put((Integer) id, lastFrame.get((Integer)id));
            lostTime.put((Integer)id, currentTime);
        }
    }


    // When we get a new frame of objects, we want to try to match them
    // to the objects from the most recent frame. If there are no matches
    // then we want to compare to objects from previous frames (although this
    // actually doesn't matter with Soar's current capabilities - 6/7/2012).
    // We also want to check whether the arm is currently moving something,
    // and if it is we want to keep broadcasting that location.
    public HashMap<Integer, ObjectInfo> newObjects(HashMap<Integer,
                                                   ObjectInfo> currentFrame)
    {
        // Make sure there are previous objects
        if(lastFrame.size() <= 0){
            Set newObjects = currentFrame.keySet();
            for(Object id : newObjects)
                currentFrame.get((Integer)id).getID();
            lastFrame = currentFrame;
            return currentFrame;
        }

        // if we just dropped an object, add it to the last frame
        if(armState == ArmState.JUST_DROPPED_OBJECT){
            lastFrame.put(heldObject.repID, heldObject);
            armState = ArmState.WAITING;
        }

        removeDarkObjects(currentFrame);

        // Keep track of which objects haven't been paired yet
        HashMap<Integer, Integer> unusedNew = new HashMap<Integer, Integer>();
        HashMap<Integer, Integer> unusedOld = new HashMap<Integer, Integer>();
        Set currentIDs = currentFrame.keySet();
        Set oldIDs = lastFrame.keySet();
        for(Object idNew : currentIDs)
            unusedNew.put((Integer)idNew, (Integer)idNew);
        for(Object idOld : oldIDs )
            unusedOld.put((Integer)idOld, (Integer) idOld);

        // Greedy search to match objects between current and last frame
        ObjectInfo objNew, objOld;
        for(int i=0; i<currentFrame.size(); i++){
            Set cNew = unusedNew.keySet();
            Set cOld = unusedOld.keySet();

            double bestMatch = 10000;
            int newID = -1;
            int oldID = -1;

            for(Object currentObj : cNew){
                Integer currentID = (Integer) currentObj;
                objNew = currentFrame.get(currentID);
                for(Object oldObj : cOld){
                    Integer pastID = (Integer) oldObj;
                    objOld = lastFrame.get(pastID);
                    double dist = xydistance(objNew.getCenter(),
                                                  objOld.getCenter());
                    double color = LinAlg.distance(objNew.avgColor(),
                                                   objOld.avgColor());
                    double score = color + dist;

                    if(dist < bestMatch //&& dist < MAX_TRAVEL_DIST){
                       && dist < MAX_TRAVEL_DIST
                       && color < MAX_COLOR_CHANGE){
                        newID = currentID;
                        oldID = pastID;
                        bestMatch = dist;
                    }
                }
            }

            // Equate the best pair
            if(newID > -1 && oldID > -1){
                unusedNew.remove(newID);
                unusedOld.remove(oldID);
                ObjectInfo obj = currentFrame.get(newID);
                obj.equateObject(lastFrame.get(oldID).repID, lastFrame.get(oldID).color);
                obj.matched = true;
            }
        }

        // See if any of the unmatched objects match lost objects
        if(unusedNew.size() > 0){
            Set unusedSet = unusedNew.keySet();
            Set lostSet = lostObjects.keySet();
            for(Object unused : unusedSet){
                Integer unusedID = (Integer) unused;
                ObjectInfo unusedObj = currentFrame.get(unusedID);
                double bestMatch = 10000;
                Integer bestID = -1;
                for(Object lost : lostSet){
                    Integer lostID = (Integer) lost;
                    if(lostObjects.containsKey(lostID)){
                        ObjectInfo lostObj = lostObjects.get(lostID);
                        double dist = xydistance(unusedObj.getCenter(),
                                                      lostObj.getCenter());
                        double color = LinAlg.distance(unusedObj.avgColor(),
                                                       lostObj.avgColor());
                        double score = color + dist;
                        if(dist < bestMatch //&& dist < MAX_TRAVEL_DIST*1.5){
                           && dist < MAX_TRAVEL_DIST
                           && color < MAX_COLOR_CHANGE){
                            bestID = lostID;
                            bestMatch = dist;
                        }
                    }
                }

                // Pair the new object to the best lost object, if one exists,
                // otherwise give it a new repID
                if(bestID > -1){
                    unusedObj.equateObject(lostObjects.get(bestID).repID,
                                           lostObjects.get(bestID).color);
                    unusedObj.matched = true;
                    lostObjects.remove(bestID);
                    lostTime.remove(bestID);


                    //double[] c = unusedObj.getCenter();
                    //System.out.println("MATCHED "+c[0]+" "+c[1]+" "+c[2]+" to "+bestID);

                }
                else{
                    unusedObj.getID();
                    //System.out.println("NEW OBJECT "+unusedObj.repID);
                }
            }
        }


        updateHistory(unusedOld);
        lastFrame = currentFrame;

        // Add in held object, if there is one.
        if(armState == ArmState.HOLDING_OBJECT){
            heldObject.resetCenter(arm.getGripperXYZRPY());
            currentFrame.put(heldObject.ufsID, heldObject);
        }

        return currentFrame;
    }
}