package abolt.classify;

import java.util.ArrayList;

import abolt.objects.BoltObject;

public interface IClassifier {
    /**
     * @param object
     * Adds the given training example to the classifier 
     */
    void add(ArrayList<Double> features, String label);
    
	/**
	 * @param object
	 * @return the label for the object, or "unknown" if not found
	 */
	ConfidenceLabel classify(ArrayList<Double> features);
	
	
	/**
	 * Clears all data in the classifier
	 */
	void clearData();
	
	
	/**
	 * Reloads all the data in the classifier
	 */
	void loadData();
}
