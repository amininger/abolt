package abolt.kinect;

import java.util.*;

import april.jmat.*;
import april.util.*;
import abolt.classify.ColorFeatureExtractor;
import abolt.arm.*;

public class ObjectTracking
{
    final double MAX_TRAVEL_DIST = 0.05;//.1;
    final double MAX_COLOR_CHANGE = 30;//40;
    final int MAX_HISTORY = 20;
    private final static double darkThreshold = .4;

    private BoltArm arm;
    private HashMap<Integer, ObjectInfo> lastFrame;
    private HashMap<Integer, ObjectInfo> lostObjects;
    private HashMap<Integer, Long> lostTime;
    private ObjectInfo heldObject;
    private boolean holdingObject;
    private boolean justDroppedObject;

    public ObjectTracking()
    {
        arm = BoltArm.getSingleton();
        lastFrame = new HashMap<Integer, ObjectInfo>();
        lostTime = new HashMap<Integer, Long>();
        lostObjects = new HashMap<Integer, ObjectInfo>();
        holdingObject = false;
        justDroppedObject = false;
    }


    // Returns false if it fails to find the object id
    public boolean movingObject(Integer id)
    {
        Integer hashID = -1;
        Set objects = lastFrame.keySet();
        for(Object obj : objects){
            if(lastFrame.get((Integer)obj).repID == id){
                hashID = (Integer) obj;
                break;
            }
        }

        if(hashID == -1) return false;

        holdingObject = true;
        heldObject = lastFrame.get(hashID);
        justDroppedObject = false;
        return true;
    }

    // Returns false if there was no tracked object to release
    public boolean releasedObject(double[] finalLocation)
    {
        if(holdingObject){

            heldObject.resetCenter(finalLocation); //resetCenter(finalLocation);
            double[] c = heldObject.getCenter();
            lastFrame.put(heldObject.repID, heldObject);
            lostObjects.remove(heldObject.ufsID);
            holdingObject = false;
            justDroppedObject = true;
            return true;
        }
        return false;
    }


    public double xydistance(double[] p1, double[] p2)
    {
        double dx = p1[0]-p2[0];
        double dy = p1[1]-p2[1];
        return Math.sqrt(dx*dx + dy*dy);
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
        if(holdingObject == true)

        // Make sure there are previous objects
        if(lastFrame.size() <= 0){
            Set newObjects = currentFrame.keySet();
            for(Object id : newObjects)
                currentFrame.get((Integer)id).getID();
            lastFrame = currentFrame;
            return currentFrame;
        }

        // if we just dropped an object, add it to the last frame
        if(justDroppedObject){
            lastFrame.put(heldObject.repID, heldObject);
            double[] c = heldObject.getCenter();
        }

        // Remove objects that are too dark
        ArrayList<Integer> dark = new ArrayList<Integer>();
        for(ObjectInfo info : currentFrame.values()){
            ArrayList<Double> colorFeatures = ColorFeatureExtractor.getFeatures(info);
            if(colorFeatures.get(0) <= darkThreshold && colorFeatures.get(1) <= darkThreshold &&
               colorFeatures.get(2) <= darkThreshold){
                dark.add(info.ufsID);
            }
        }
        for(Integer i : dark)
            currentFrame.remove(i);

        // Keep track of which objects haven't been paired yet
        HashMap<Integer, Integer> unusedNew = new HashMap<Integer, Integer>();
        HashMap<Integer, Integer> unusedOld = new HashMap<Integer, Integer>();
        Set currentIDs = currentFrame.keySet();
        Set oldIDs = lastFrame.keySet();
        for(Object idNew : currentIDs){
            unusedNew.put((Integer)idNew, (Integer)idNew);
        }
        //System.out.print("Old IDs: ");
        for(Object idOld : oldIDs ){
            //System.out.print(lastFrame.get((Integer)idOld).repID+", ");
            unusedOld.put((Integer)idOld, (Integer) idOld);
        }
        //System.out.println();

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
                    double[] c = objNew.getCenter();
                    double[] c2 = objOld.getCenter();

                    if(dist < bestMatch
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
                        if(dist < bestMatch
                           && dist < MAX_TRAVEL_DIST*2
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
                }
                else
                    unusedObj.getID();
            }
        }

        // Increment how long each of the lost objects has been around and
        // discard ones that are too old.
        Set lost = lostObjects.keySet();
        ArrayList<Integer> toRemove = new ArrayList<Integer>();
        long currentTime = TimeUtil.utime();
        for(Object lostObj : lost){
            Integer id = (Integer) lostObj;
            int time  = (int)((currentTime -lostTime.get(id))*1000000);
            if(time >= MAX_HISTORY){
                toRemove.add(id);
                lostTime.remove(id);
            }
        }
        for(Integer id : toRemove)
            lostObjects.remove(id);

        // Add lost objects to history
        if(unusedOld.size() > 0){
            Set old = unusedOld.keySet();
            for(Object id : old){
                lostObjects.put((Integer) id, lastFrame.get((Integer)id));
                lostTime.put((Integer)id, currentTime);
            }
        }

        // Set these objects as the previous objects
        lastFrame = currentFrame;

        // Add in held object, if there is one.
        if(holdingObject){
            double[] gripperPosition = arm.getGripperXYZRPY();

            double[] c1 = heldObject.center;
            heldObject.resetCenter(gripperPosition);
            currentFrame.put(heldObject.ufsID, heldObject);
            double[] c2 = heldObject.center;
        }


        return currentFrame;
    }
}