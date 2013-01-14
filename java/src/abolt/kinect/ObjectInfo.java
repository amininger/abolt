package abolt.kinect;

import java.awt.*;
import java.util.*;
import abolt.util.*;

public class ObjectInfo{

    private int color;
    public int repID;
    public int ufsID;
    private double[] center;
    private int[] avgColor; // Red, Green, Blue
    public int[] sumColor;

    public ArrayList<double[]> points;

    public ObjectInfo(){
        center = null;
    }

    /** Create a new object with info about it. Objects begin with a single point.**/
    public ObjectInfo(int color, int id, double[] point)
    {
        this.color = color;
        this.ufsID = id;

        Color c = new Color((int) point[3]);
        sumColor = new int[]{c.getRed(), c.getGreen(), c.getBlue()};
        points = new ArrayList<double[]>();
        points.add(point);
    }

    public void createRepID()
    {
        this.repID = SimUtil.nextID();
    }

    /** Add a new point to this object. **/
    public void update(double[] point)
    {
        Color c = new Color((int)point[3]);
        sumColor[0] += c.getRed();
        sumColor[1] += c.getGreen();
        sumColor[2] += c.getBlue();

        this.points.add(point);
    }

    /** Get the center of the object (mean x, y,z). **/
    public double[] getCenter()
    {
        if(center == null){
            center = new double[3];
            double[] min = new double[]{1000, 1000, 1000};
            double[] max = new double[]{-1000, -1000, -1000};
            for(double[] p : points){
                for(int i=0; i<3; i++){
                    if(p[i]<min[i])
                        min[i] = p[i];
                    if(p[i] > max[i])
                        max[i] = p[i];
                }
            }
            for(int i=0; i<3; i++){
                center[i] = (min[i]+max[i])/2.0;
            }
            //center = KUtils.getWorldCoordinates(center);
        }
        return center;
    }


    public void resetCenter(double[] newCenter)
    {
        // Change the locationof all the points in the object
        double[] translation = new double[3];
        for(int i=0; i<translation.length; i++){
            translation[i] = newCenter[i]-center[i];
        }
        for(double[] p : points)
            for(int i=0; i<translation.length; i++)
                p[i] += translation[i];

        center = newCenter;
    }

    /** Get the average color of the object as an array [r, g, b]. **/
    public int[] avgColor()
    {
        if(avgColor == null){
            avgColor = new int[sumColor.length];
            for(int i=0; i<avgColor.length; i++){
                avgColor[i] = sumColor[i]/points.size();
            }
        }
        return avgColor;
    }


    /** Say this object is the same as a past one by giving it the old object's
        ID. Eventually this method might allow us to store the old object in the
        history of this object? **/
    public void equateObject(int newID, int[] newColor)
    {
        repID = newID;
        avgColor = newColor;
    }


}
