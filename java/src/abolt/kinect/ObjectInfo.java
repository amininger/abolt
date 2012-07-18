package abolt.kinect;

import java.io.*;
import java.nio.*;
import javax.swing.*;
import java.awt.*;
import java.util.*;
import java.awt.image.*;

import lcm.lcm.*;

import april.vis.*;
import april.jmat.*;
import april.util.UnionFindSimple;

import abolt.bolt.Bolt;
import abolt.classify.*;
import abolt.classify.Features.FeatureCategory;
import abolt.lcmtypes.*;
import abolt.objects.ISimBoltObject;
import abolt.util.*;

public class ObjectInfo{

    private int color;
    public int repID;
    public int ufsID;
    private double[] center;
    private int[] avgColor; // Red, Green, Blue
    public int[] sumColor;
    public BufferedImage image = null;
    public Rectangle projBBox = null;
    public ISimBoltObject createdFrom = null;

    public ArrayList<double[]> points;
    private HashMap<FeatureCategory, ArrayList<Double> > features;

    public ObjectInfo(){
        center = null;
    	features = new HashMap<FeatureCategory, ArrayList<Double> >();
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
    	features = new HashMap<FeatureCategory, ArrayList<Double>>();
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

    public ArrayList<Double> getFeatures(FeatureCategory cat){
    	if(features.containsKey(cat)){
    		return features.get(cat);
    	} else {
    		ArrayList<Double> fts = Features.getFeatures(cat, points);
    		features.put(cat, fts);
    		return fts;
    	}
    }
    
    public void addFeatures(FeatureCategory cat, ArrayList<Double> features){
		this.features.put(cat, features);
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
            // For Aaron
            center = Bolt.getCamera().getWorldCoords(center);
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

    public static BufferedImage getImage(ArrayList<double[]> points, Rectangle projBBox){
    	BufferedImage image;
		int minX = Integer.MAX_VALUE, maxX = Integer.MIN_VALUE;
		int minY = Integer.MAX_VALUE, maxY = Integer.MIN_VALUE;
		IBoltCamera camera = Bolt.getCamera();
		for(double[] pt : points){
			int[] pixel = camera.getPixel(pt);
			if(pixel == null){
				continue;
			}
			minX = (pixel[0] < minX ? pixel[0] : minX);
			maxX = (pixel[0] > maxX ? pixel[0] : maxX);
			minY = (pixel[1] < minY ? pixel[1] : minY);
			maxY = (pixel[1] > maxY ? pixel[1] : maxY);
		}
		int margin = 5;
		if(projBBox != null){
			projBBox.setBounds(minX - margin, minY - margin,
                               maxX - minX + 1 + margin*2,
                               maxY - minY + 1 + margin*2);
		}
		image = new BufferedImage((maxX - minX + 1) + 2*margin,
                                  (maxY - minY + 1) + 2*margin,
                                  BufferedImage.TYPE_3BYTE_BGR);
		for(int i = 0; i < points.size(); i++){
			int[] pixel = Bolt.getCamera().getPixel(points.get(i));
			if(pixel == null){
				continue;
			}
			try{
				Color c =  new Color((int)points.get(i)[3]);
				Color rc = new Color(c.getBlue(), c.getGreen(), c.getRed());
    			image.setRGB(pixel[0]+margin-minX,
                             pixel[1]+margin-minY,
                             rc.getRGB());
			} catch (Exception e){
                e.printStackTrace();
			}
		}
    	return image;
    }

    public Rectangle getProjectedBBox(){
    	if(projBBox == null){
    		getImage();
    	}
    	return projBBox;
    }

    public BufferedImage getImage(){
    	if(image == null){
    		projBBox = new Rectangle();
    		image = getImage(points, projBBox);
    	}
    	return image;
    }
}
