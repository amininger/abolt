package abolt.classify;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/**
 * @author Aaron
 * This class extends KNN for shapes, because when new shapes are added
 * this also stores flipped versions of the shape
 */
public class ShapeKNN extends KNN {

	public ShapeKNN(int Km, int dim, String datafile, double maxDistance) {
		super(Km, dim, datafile, maxDistance);
	}
	
	@Override 
	public void add(KNNPoint pt) {
    	if(pt.getCoords().size() != dim || pt.getCoords().size() == 0){
    		return;
    	}
    	if(pt.getLabel() == null){
    		System.out.println("NULL label");
    		return;
    	}
    	
    	List<Double> coords = pt.getCoords();
    	
    	double first = coords.get(0);
    	ArrayList<Double> pt1 = new ArrayList<Double>();
    	ArrayList<Double> pt2 = new ArrayList<Double>();
    	
    	for(int i = 1; i < coords.size(); i++){
    		if (i-1 < (dim-1)/2)
    	          pt1.add(coords.get(i));
    	    else
    	          pt2.add(coords.get(i));
    	}
    	
    	// Regular
    	coords = new ArrayList<Double>();
    	coords.add(first);
    	coords.addAll(pt1);
    	coords.addAll(pt2);
    	data.add(new KNNPoint(coords, pt.getLabel()));

    	// Vertically Flipped
    	coords = new ArrayList<Double>();
    	coords.add(first);
    	coords.addAll(pt2);
    	coords.addAll(pt1);
    	data.add(new KNNPoint(coords, pt.getLabel()));

        Collections.reverse(pt1);
        Collections.reverse(pt2);
        
        // Horizontally Flipped
    	coords = new ArrayList<Double>();
    	coords.add(first);
    	coords.addAll(pt1);
    	coords.addAll(pt2);
    	data.add(new KNNPoint(coords, pt.getLabel()));

    	// Horizontally and Vertically Flipped
    	coords = new ArrayList<Double>();
    	coords.add(first);
    	coords.addAll(pt2);
    	coords.addAll(pt1);
    	data.add(new KNNPoint(coords, pt.getLabel()));
	}
}
