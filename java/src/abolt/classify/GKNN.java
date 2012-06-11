package abolt.classify;

import java.util.*;

/** K-nearest neighbor classifier using a guassian-like
 *  weighting function when considering the importance
 *  of the neighbors
 */
public class GKNN implements IClassifier
{
    ArrayList<CPoint> points = new ArrayList<CPoint>();
    class CPoint
    {
        public String label;
        public double[] coords;

        public CPoint(double[] coords_, String label_)
        {
            label = label_;
            coords = coords_;
        }
    }

    // Number of nearest neighbors
    int k;

    public GKNN(int k_)
    {
        k = k_;
    }

    /** Add a training example to the classifier */
    public void add(ArrayList<Double> features, String label)
    {
        if (label == null)
            return;
        double[] dfeatures = ArrayList.toArray(new double[0]);
        add(dfeatures, label);
    }

    public void add(double[] features, String label)
    {
        if (label == null)
            return;
        points.add(features, label);
    }

    /** Return a confidence label given the features */
    ConfidenceLabel classify(ArrayList<Double> features)
    {
        // Find the k nearest neighbors
        //
        // Evaluate the neighbors based on the weights
        //
        // Return
        return null;    // XXX Not yet implemented
    }

    /** Clear the classifier information */
    void clearData()
    {
        points.clear();
    }

    /** Load in the classifier data */
    void loadData()
    {
    }
}
