package abolt.classify;

import java.awt.Color;
import java.util.*;
import abolt.collision.*;
import april.jmat.LinAlg;
import april.sim.SimObject;
import april.vis.*;

/**
 * @author aaron
 * @purpose Creates fake features for simulator objects when used in classifiers
 */
public class SimFeatures {	
	// Mapping from a color string to a color object
	private static HashMap<String, Color> colorValues;
	static{
		colorValues = new HashMap<String, Color>();
		colorValues.put("red",    new Color(255, 40, 20));
		colorValues.put("orange", new Color(255, 128, 20));
		colorValues.put("yellow", new Color(255, 255, 50));
		colorValues.put("green",  new Color(40, 255, 40));
		colorValues.put("blue",   new Color(40, 40, 255));
		colorValues.put("purple", new Color(255, 0, 255));
	}
	public static Color getColorValue(String color){
		return colorValues.get(color.toLowerCase());
	}
	
	// Mapping from a size string to a scale value
	private static HashMap<String, Double> sizeValues;
	static{
		sizeValues = new HashMap<String, Double>();
		sizeValues.put("small", .6);
		sizeValues.put("medium", 1.0);
		sizeValues.put("large", 1.5);
	}
	public static Double getSizeValue(String size){
		return sizeValues.get(size.toLowerCase());
	}
	
	// Mapping from a shape string to a sim object with the given shape
	public static VisObject constructVisObject(String shape, Color color, double scale){
		shape = shape.toLowerCase();
		VzMesh.Style style = new VzMesh.Style(color);
		VisObject obj;
		if(shape.equals("square")){
			obj = new VzBox(style);
			scale *= 1.8;
		} else if(shape.equals("cylinder")){
			obj = new VzCylinder(style);
		} else if(shape.equals("sphere")){
			obj = new VzSphere(style);
		} else {
			return null;
		}
		return new VisChain(LinAlg.scale(scale), obj);
	}
	
	public static Shape constructShape(String shapeStr, double scale){
		if(shapeStr.equals("square")){
			scale *= 2;
			return new BoxShape(scale, scale, scale);
		} else if(shapeStr.equals("triangle")){
			double[][] v = new double[][]{
					new double[]{-.9, -.4},
					new double[]{.9, -.4},
					new double[]{0, .6}};
			
			
//			double[][] v = new double[][]{
//					new double[]{0, .5},
//					new double[]{-.5, .1},
//					new double[]{-.3, -.2},
//					new double[]{.3, -.2},
//					new double[]{.5, .1}};
//			return new FlatPolygonShape(v, scale * 4);
			return ShapeFactory.constructFlatPolygon(LinAlg.scale(v, scale), scale/2);
		} else {
			return new SphereShape(scale);
		}
	}
}
