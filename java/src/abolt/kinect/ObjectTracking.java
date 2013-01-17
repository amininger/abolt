package abolt.kinect;

import java.util.*;

import april.jmat.*;
import april.jmat.geom.*;
import april.util.*;
import abolt.classify.ColorFeatureExtractor;
import abolt.arm.*;

public class ObjectTracking
{
    final double MAX_TRAVEL_DIST = 0.07;//.1;
    final double MAX_COLOR_CHANGE = 60;//40;
    final int MAX_HISTORY = 10;
    final int MAX_JUST_DROPPED = 5;

    public BoltArm arm; //XXXXXXXXXXXXXXXXXXXx
    private HashMap<Integer, ObjectInfo> lastFrame;
    private HashMap<Integer, ObjectInfo> lostObjects;
    private HashMap<Integer, Long> lostTime;
    private ObjectInfo heldObject;
    private int justDroppedCount;

    enum ArmState
    {  GRABBING_OBJECT, HOLDING_OBJECT, DROPPING_OBJECT,
       JUST_DROPPED_OBJECT, WAITING,
    }
    private ArmState armState = ArmState.WAITING;


    /** The object tracking class compares objects between frames and keeps a
     *  short memory of lost objects. It attempts to reassign the correct id to
     *  objects from one frame to the next. Additionally, it keeps track of
     *  objects that have been picked up by the hand and tries to maintain their
     *  ID when they are set down.
     */
    public ObjectTracking()
    {
        arm = BoltArm.getSingleton();
        lastFrame = new HashMap<Integer, ObjectInfo>();
        lostTime = new HashMap<Integer, Long>();
        lostObjects = new HashMap<Integer, ObjectInfo>();
    }

    /** When the arm begins its "GRAB" state, note which repID is being picked up.
     *  @param id is the repID of the object the arm will try to grab.
     **/
    public void armGrabbing(Integer id)
    {
        if(armState != ArmState.GRABBING_OBJECT){
            Integer hashID = -1;
            Set objects = lastFrame.keySet();
            for(Object obj : objects){
                if(lastFrame.get((Integer)obj).repID == id){
                    hashID = (Integer) obj;
                    break;
                }
            }

            heldObject = lastFrame.get(hashID);
            if(heldObject != null){
                armState = ArmState.GRABBING_OBJECT;
                removeRepIDFromHistory(heldObject.repID);
            }
        }
    }

    /** When the arm is in "DROP" state, set the expected final location of the
     *  grabbed object to the arm's goal destination.
     *  @param finalLocation is the goal drop location for the object.
     **/
    public void armDropping(double[] finalLocation)
    {
        if(heldObject != null){
            armState = ArmState.DROPPING_OBJECT;
            heldObject.resetCenter(finalLocation);
        }
    }

    /** When the arm is in "WAIT" state, do different actions depending upon
     *  whether it was just grabbing an object (now it should be holding it) or
     *  if it was just dropping an object.
     **/
    public void armWaiting()
    {
        if(armState == ArmState.GRABBING_OBJECT
           || armState == ArmState.HOLDING_OBJECT){
            armState = ArmState.HOLDING_OBJECT;
            removeRepIDFromHistory(heldObject.repID);
        }
        else if(armState == ArmState.DROPPING_OBJECT){
            armState = ArmState.JUST_DROPPED_OBJECT;
            justDroppedCount = 0;
        }
        else if (armState == ArmState.JUST_DROPPED_OBJECT){
            if(justDroppedCount < MAX_JUST_DROPPED)
                armState = ArmState.JUST_DROPPED_OBJECT;
            else{
                armState = ArmState.WAITING;
                heldObject = null;
            }
        }
        else
            armState = ArmState.WAITING;
    }

    /** When the arm hits a "FAIL" state then no object was picked up. **/
    public void armFailed()
    {
        heldObject = null;
        armState = ArmState.WAITING;
    }

    /** Calculate the Euclidean distance in the xy-plane between two points.
     *  @param p1 the first point
     *  @param p2 the second point
     **/
    public double xydistance(double[] p1, double[] p2)
    {
        double dx = p1[0]-p2[0];
        double dy = p1[1]-p2[1];
        return Math.sqrt(dx*dx + dy*dy);
    }

    /** Remove items in history that have been there too long and add newly
     *  lost objects to the history of lost objects.
     *  @param unusedOld is a list of IDs of objects from the last frame not matched
     *         to an object in the current frame.
     **/
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


    /** Check all lost objects and objects from the last frame for a given repID,
     *  and remove them if they are found.
     *  @param id is the repID being removed.
     **/
    public void removeRepIDFromHistory(Integer id)
    {
        // Remove held object from the last frame
        Set objects = lastFrame.keySet();
        ArrayList<Integer> toRemove = new ArrayList<Integer>();
        ObjectInfo last;
        for(Object obj : objects){
	    last = lastFrame.get((Integer)obj);
            if(last != null && last.repID == id){
                toRemove.add((Integer) obj);
            }
        }
        for(Integer i : toRemove)
            lastFrame.remove(i);

        // Remove held object from the lost object list
        objects = lostObjects.keySet();
        toRemove = new ArrayList<Integer>();
        ObjectInfo lost;
        for(Object obj : objects){
	    lost = lostObjects.get((Integer)id);
            if(lost != null && lost.repID == id){
                toRemove.add((Integer) obj);
            }
        }
        for(Integer i : toRemove){
            lostObjects.remove(i);
            lostTime.remove(i);
        }
    }


    /** When we get a new frame of objects, we want to try to match them
     *  to the objects from the most recent frame. If there are no matches
     *  then we want to compare to objects from previous frames (although this
     *  actually doesn't matter with Soar's current capabilities - 6/7/2012).
     *  We also want to check whether the arm is currently moving something,
     *  and if it is we want to keep broadcasting that location. **/
    public HashMap<Integer, ObjectInfo> newObjects(HashMap<Integer,
                                                   ObjectInfo> currentFrame)
    {
        //System.out.println("==========="+armState+"============"+(heldObject!=null));

        if(armState == ArmState.JUST_DROPPED_OBJECT)
            justDroppedCount ++;

        // Make sure there are previous objects
        /*if(lastFrame.size() <= 0){
            for(Object id : currentFrame.values()){
                ObjectInfo oi = (ObjectInfo) id;
                oi.createRepID();
            }
            lastFrame = currentFrame;
            if(heldObject != null){
                heldObject.resetCenter(arm.getGripperXYZRPY());
                currentFrame.put(heldObject.ufsID, heldObject);
            }
            return currentFrame;
        }*/

        // Keep track of which objects haven't been paired yet
        HashMap<Integer, Integer> unusedNew = new HashMap<Integer, Integer>();
        HashMap<Integer, Integer> unusedOld = new HashMap<Integer, Integer>();
        Set currentIDs = currentFrame.keySet();
        Set oldIDs = lastFrame.keySet();
        for(Object idNew : currentIDs)
            unusedNew.put((Integer)idNew, (Integer)idNew);
        for(Object idOld : oldIDs )
            unusedOld.put((Integer)idOld, (Integer) idOld);

        // if we just dropped an object, Widen the range of the match area,
        // try to match it before anything else.
        boolean unmatchedDroppedObject = false;
        if(heldObject != null ){
            double[] pos = heldObject.getCenter();
            //System.out.printf("%.4f, %.4f, %.4f\n", pos[0], pos[1], pos[2]);
            double bestMatch = 100000;
            int bestID = -1;

            Set cNew = unusedNew.keySet();
            for(Object currentID : cNew){
                ObjectInfo objNew = currentFrame.get((Integer)currentID);
                double dist = xydistance(objNew.getCenter(),
                                         heldObject.getCenter());
                double color = LinAlg.distance(objNew.avgColor(),
                                               heldObject.avgColor());
                if(dist < bestMatch
                   && dist < MAX_TRAVEL_DIST
                   && color < MAX_COLOR_CHANGE){
                    bestID = (Integer)currentID;
                    bestMatch = dist;
                }
            }

            // Equate the best pair
            if(bestID > -1){
                unusedNew.remove(bestID);
                ObjectInfo obj = currentFrame.get(bestID);
                obj.equateObject(heldObject.repID, heldObject.avgColor());
                Integer toRemove = -1;
                for(Object id : unusedOld.keySet()){
                    if(lastFrame.get((Integer)id).repID == heldObject.repID)
                        toRemove = (Integer) id;
                }
                if(toRemove > -1)
                    unusedOld.remove(toRemove);

                removeRepIDFromHistory(heldObject.repID);
                //System.out.println("Found a match!");
                if(armState == ArmState.JUST_DROPPED_OBJECT){
                    //System.out.println("Setting held object to null");
                    heldObject = null;
                }
            }
            else
                unmatchedDroppedObject = true;
        }

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
                    if(objNew != null && objOld != null){
                        double dist = xydistance(objNew.getCenter(),
                                                 objOld.getCenter());
                        double color = LinAlg.distance(objNew.avgColor(),
                                                       objOld.avgColor());
                        double score = color + dist;

                        if(dist < bestMatch
                           && dist < MAX_TRAVEL_DIST
                           && color < MAX_COLOR_CHANGE){
                            newID = currentID;
                            oldID = pastID;
                            bestMatch = dist;
                        }
                    }
                }
            }

            // Equate the best pair
            if(newID > -1 && oldID > -1){
                unusedNew.remove(newID);
                unusedOld.remove(oldID);
                ObjectInfo obj = currentFrame.get(newID);
                obj.equateObject(lastFrame.get(oldID).repID, lastFrame.get(oldID).avgColor());
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
                    ObjectInfo lostObj = lostObjects.get(lostID);
                    if(lostObj != null && unusedObj != null){

                        double dist = xydistance(unusedObj.getCenter(),
                                                 lostObj.getCenter());
                        double color = LinAlg.distance(unusedObj.avgColor(),
                                                       lostObj.avgColor());
                        double score = color/441 + dist;
                        if(dist < bestMatch
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
                                           lostObjects.get(bestID).avgColor());
                    lostObjects.remove(bestID);
                    lostTime.remove(bestID);
                }
                else
                    unusedObj.createRepID();
            }
        }

        updateHistory(unusedOld);
        lastFrame = currentFrame;

        // Add in held object, if there is one.
        if((heldObject != null && armState == ArmState.HOLDING_OBJECT)
           || unmatchedDroppedObject){
            if(armState == ArmState.HOLDING_OBJECT)
                heldObject.resetCenter(arm.getGripperXYZRPY());

            currentFrame.put(heldObject.ufsID, heldObject);
        }

/*
        System.out.println("Current Objects:");
        for(ObjectInfo info : currentFrame.values()){
            double[] c = info.getCenter();
            System.out.printf("\t%d: (%.4f, %.4f, %.4f)\n", info.repID, c[0], c[1], c[2]);
        }
        System.out.println("Lost Objects:");
        for(ObjectInfo info : .values()){
            double[] c = info.getCenter();
            System.out.printf("\t%d: (%.4f, %.4f, %.4f)\n", info.repID, c[0], c[1], c[2]);
        }
*/
        return currentFrame;
    }
}