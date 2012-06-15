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
import abolt.classify.Features;
import abolt.classify.Features.FeatureCategory;
import abolt.lcmtypes.*;
import abolt.objects.ISimBoltObject;
import abolt.util.*;

public class ObjectInfo{

    public double numPoints;
    public int color;
    public int repID;
    public int ufsID;
    public double[] center;
    public int[] sumColor;
    public double leftmost;
    public double rightmost;
    public double uppermost;
    public double lowermost;
    public BufferedImage image = null;
    public boolean matched;
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
//        Random r = new Random();
        this.numPoints = 1;
        this.color = color;
        this.ufsID = id;
        // XXXX - Need to transform everything with respect to the ground
        // plane and the kinect
        this.leftmost = point[0];
        this.rightmost = point[0];
        this.uppermost = point[1];
        this.lowermost = point[1];
        this.matched = false;
        Color c = new Color((int) point[3]);
        sumColor = new int[]{c.getRed(), c.getBlue(), c.getGreen()};

        this.points = new ArrayList<double[]>();
        this.points.add(point);
    	features = new HashMap<FeatureCategory, ArrayList<Double> >();
    }

    public void getID()
    {
        this.repID = SimUtil.nextID();
    }

    /** Add a new point to this object. **/
    public void update(double[] point){
        if(this.leftmost > point[0])
            this.leftmost = point[0];
        if(this.rightmost < point[0])
            this.rightmost = point[0];
        if(this.uppermost > point[1])
            this.uppermost = point[1];
        if(this.lowermost < point[1])
            this.lowermost = point[1];

        this.numPoints ++;
        Color c = new Color((int)point[3]);
        sumColor[0] += c.getRed();
        sumColor[1] += c.getBlue();
        sumColor[2] += c.getGreen();

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
            center = KUtils.getWorldCoordinates(center);
        }
        return center;
//        return new double[]{center[1], center[0], center[2]};
    }


    public void resetCenter(double[] newCenter)
    {
        // Change the locationof all the points in the object
        double[] translation = new double[3];
        for(int i=0; i<translation.length; i++){
            translation[i] = newCenter[i]-center[i];
        }
        for(double[] p : points)
            LinAlg.plusEquals(p, translation);

        // Update center
        center = newCenter;
    }

    /** Get the average color of the object as an array [r, g, b]. **/
    public double[] avgColor()
    {
        double[] avg = new double[sumColor.length];
        for(int i=0; i<avg.length; i++){
            avg[i] = sumColor[i]/numPoints;
        }
        return avg;
    }

    /** Given a hashmap of objects, find the one that is most similar to this
        object. The most similar one will be the object that has the closest
        center and with a mean color that is within a threshold of this object.**/
    // XXX - probably want to take complete feature vector into account, not only colors
    public int mostSimilar(HashMap<Integer, ObjectInfo> objects, HashMap<Integer, Integer> alreadyAssigned)
    {
        int best = -1;
        double minDist = .1;
        double minColorDist = 30;

        Collection c = objects.values();
        for(Iterator itr = c.iterator(); itr.hasNext(); ){
            ObjectInfo obj2 = (ObjectInfo)itr.next();
            double centerDist = LinAlg.distance(getCenter(), obj2.getCenter());
            double colorDist = LinAlg.distance(avgColor(), obj2.avgColor());
            boolean okay = true;
            if(alreadyAssigned != null)
            	if(alreadyAssigned.containsKey(obj2.ufsID)) okay = false;
            if (okay && centerDist < minDist && colorDist < minColorDist){
                minDist = centerDist;
                best = obj2.ufsID;
            }
        }
        return best;
    }

    /** Say this object is the same as a past one by giving it the old object's
        ID. Eventually this method might allow us to store the old object in the
        history of this object? **/
    public void equateObject(int newID, int newColor)
    {
        repID = newID;
        color = newColor;
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
			projBBox.setBounds(minX - margin, minY - margin, maxX - minX + 1 + margin*2, maxY - minY + 1 + margin*2);
		}
		image = new BufferedImage((maxX - minX + 1) + 2*margin, (maxY - minY + 1) + 2*margin, BufferedImage.TYPE_3BYTE_BGR);
		for(int i = 0; i < points.size(); i++){
			int[] pixel = Bolt.getCamera().getPixel(points.get(i));
			if(pixel == null){
				continue;
			}
			try{
				Color c =  new Color((int)points.get(i)[3]);
				Color rc = new Color(c.getBlue(), c.getGreen(), c.getRed());
    			image.setRGB(pixel[0]+margin-minX, pixel[1]+margin-minY, rc.getRGB());
			} catch (Exception e){
				//System.out.println("Out of Bounds pixel in ObjectInfo: " + pixel[0] + ", " + pixel[1]);
				//System.out.println(points.get(i)[0] + ", " + points.get(i)[1] + ", " + points.get(i)[2]);
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
