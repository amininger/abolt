package abolt.classify;

import java.awt.Color;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;

import april.jmat.LinAlg;
import april.sim.SimObject;
import april.vis.*;

public class SimFeatures {
	// COLOR
	// Mapping from a color string to the features used in the classifier
	private static HashMap<String, Double[]> colorFeatures;
	static{
		colorFeatures = new HashMap<String, Double[]>();
		colorFeatures.put("red",    new Double[]{0.90, 0.29, 0.47, 0.93, 0.68, 0.89});
		colorFeatures.put("orange", new Double[]{0.90, 0.49, 0.39, 0.18, 0.57, 0.90});
		colorFeatures.put("yellow", new Double[]{0.93, 0.91, 0.60, 0.16, 0.37, 0.94});
		colorFeatures.put("green",  new Double[]{0.63, 0.86, 0.47, 0.26, 0.46, 0.87});
		colorFeatures.put("blue",   new Double[]{0.42, 0.55, 0.83, 0.61, 0.49, 0.83});
		colorFeatures.put("purple", new Double[]{0.69, 0.54, 0.83, 0.75, 0.34, 0.83});
	}
	public static ArrayList<Double> getColorFeatures(String color){
		return new ArrayList<Double>(Arrays.asList(colorFeatures.get(color.toLowerCase())));
	}
	
	// Mapping from a color string to a color object
	private static HashMap<String, Color> colorValues;
	static{
		colorValues = new HashMap<String, Color>();
		colorValues.put("red",    Color.red);
		colorValues.put("orange", Color.orange);
		colorValues.put("yellow", Color.yellow);
		colorValues.put("green",  Color.green);
		colorValues.put("blue",   Color.blue);
		colorValues.put("purple", new Color(192, 0, 192));
	}
	public static Color getColorValue(String color){
		return colorValues.get(color.toLowerCase());
	}
	
	// SIZE
	// Mapping from a size string to the features used by a classifier
	private static HashMap<String, Double[]> sizeFeatures;
	static{
		sizeFeatures = new HashMap<String, Double[]>();
		sizeFeatures.put("small",  new Double[]{0.102, 0.023});
		sizeFeatures.put("medium", new Double[]{0.152, 0.033});
		sizeFeatures.put("large",  new Double[]{0.235, 0.051});
	}
	public static ArrayList<Double> getSizeFeatures(String size){
		return new ArrayList<Double>(Arrays.asList(sizeFeatures.get(size.toLowerCase())));
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
	
	// SHAPE
	// Mapping from a shape string to the features used by a classifier
	private static HashMap<String, Double[]> shapeFeatures;
	static{
		shapeFeatures = new HashMap<String, Double[]>();
		shapeFeatures.put("square",   new Double[]{1.0,  0.31,  0.19,  0.116, 0.058, 0.048, 0.212, 0.47,  0.53,  0.27,  0.019, 0.01, 0.116, 0.232, 0.348});
		shapeFeatures.put("cylinder", new Double[]{1.15, 0.39,  0.21,  0.052, 0.0,   0.0,   0.123, 0.335, 0.335, 0.052, 0.026, 0.0,  0.026, 0.026, 0.335});
		shapeFeatures.put("sphere",   new Double[]{1.0,  0.336, 0.107, 0.015, 0.0,   0.015, 0.107, 0.321, 0.347, 0.118, 0.027, 0.0,  0.019, 0.103, 0.317});
	}
	public static ArrayList<Double> getShapeFeatures(String shape){
		return new ArrayList<Double>(Arrays.asList(shapeFeatures.get(shape.toLowerCase())));
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
}
