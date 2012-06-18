package abolt.classify;

import java.util.HashMap;
import java.util.Map;

import abolt.classify.Features.FeatureCategory;


/**
 * @author Aaron Mininger
 * This class is the central location to specify the labels used by soar for visual labels
 */
public class SoarSymbols {
	static Map<String, String> shapeSymbols;
	static Map<String, String> sizeSymbols;
	static Map<String, String> colorSymbols;
	
	/**
	 * @param color
	 * @return Returns the internal symbol used by soar for the given color
	 */
	public static String getColorSymbol(String color){
		return colorSymbols.get(color);
	}
	
	/**
	 * @param shape
	 * @return Returns the internal symbol used by soar for the given shape
	 */
	public static String getShapeSymbol(String shape){
		return shapeSymbols.get(shape);
	}
	
	/**
	 * @param size
	 * @return Returns the internal symbol used by soar for the given size
	 */
	public static String getSizeSymbol(String size){
		return sizeSymbols.get(size);
	}
	
	public static String getSymbol(FeatureCategory type, String feature){
		switch(type){
		case COLOR:
			return getColorSymbol(feature);
		case SHAPE:
			return getShapeSymbol(feature);
		case SIZE:
			return getSizeSymbol(feature);
		}
		return null;
	}
	
	static{
		shapeSymbols = new HashMap<String, String>();
		shapeSymbols.put("triangle", "shape-triangle");
		shapeSymbols.put("rectangle", "shape-rectangle");
		shapeSymbols.put("triangular", "shape-triangle");
		shapeSymbols.put("rectangular", "shape-rectangle");
		shapeSymbols.put("square", "shape-square");
		shapeSymbols.put("l-shaped", "shape-lblock");
		shapeSymbols.put("t-shaped", "shape-tblock");
		shapeSymbols.put("arch", "shape-arch");
		//shapeSymbols.put("doughnut", "shape-doughnut");
		shapeSymbols.put("half-cylinder", "shape-halfcylinder");
		shapeSymbols.put("cylinder", "shape-cylinder");
		shapeSymbols.put("sphere", "shape-sphere");
		
		colorSymbols = new HashMap<String, String>();
		colorSymbols.put("red", "color-red");
		colorSymbols.put("orange", "color-orange");
		colorSymbols.put("yellow", "color-yellow");
		colorSymbols.put("green", "color-green");
		colorSymbols.put("blue", "color-blue");
		colorSymbols.put("purple", "color-purple");
		
		sizeSymbols = new HashMap<String, String>();
		sizeSymbols.put("large", "size-large");
		sizeSymbols.put("medium", "size-medium");
		sizeSymbols.put("small", "size-small");
	}
}
