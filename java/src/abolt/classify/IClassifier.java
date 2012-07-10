package abolt.classify;

import java.util.*;

import abolt.objects.*;

public interface IClassifier {
    /**
     * @param object
     * Adds the given training example to the classifier.
     * Automatically creates the point for it.
     * XXX Legacy code. To be removed. Preferred:
     * add(CPoint);
     */
    void add(ArrayList<Double> features, String label);

    /** Add an existing point to the classifier */
    void add(CPoint point);

    /** Remove and return the last point, if it exists */
    CPoint removeLast();

	/**
	 * @param object
	 * @return the label for the object, or "unknown" if not found
	 */
    Classifications classify(ArrayList<Double> features);


	/**
	 * Clears all data in the classifier
	 */
	void clearData();


	/**
	 * Reloads all the data in the classifier
	 */
	void loadData();
}
