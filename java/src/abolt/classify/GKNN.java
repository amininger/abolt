package abolt.classify;

import java.util.*;

import april.jmat.*;

import abolt.util.*;

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

    class CPointComparator implements Comparator<CPoint>
    {
        double[] goal;

        public CPointComparator(double[] goal_)
        {
            goal = LinAlg.copy(goal_);
        }

        public int compare(CPoint a, CPoint b)
        {
            double da = LinAlg.distance(a.coords, goal);
            double db = LinAlg.distance(b.coords, goal);

            if (da > db)
                return 1;
            else if (da < db)
                return -1;
            else
                return 0;
        }
    }

    // Number of nearest neighbors
    int k;

    // Covariance parameter
    double[][] P;

    /** Initialize the KNN
     *  @param k_       The max number of neighbors considered
     *  @param P_       A covariance matrix parameter affecting weighting
     */
    public GKNN(int k_, double[][] P_)
    {
        k = k_;
        P = P_;
    }

    /** Add a training example to the classifier */
    public void add(ArrayList<Double> features, String label)
    {
        if (label == null)
            return;
        double[] dfeatures = BoltMath.toArray(features);
        add(dfeatures, label);
    }

    public void add(double[] features, String label)
    {
        if (label == null)
            return;
        points.add(new CPoint(features, label));
    }

    /** Return a confidence label given the features */
    public ConfidenceLabel classify(ArrayList<Double> features)
    {
        // Find the k nearest neighbors
        //
        // Evaluate the neighbors based on the weights
        //
        // Return
        return null;    // XXX Not yet implemented
    }

    /** Return a list of classifications ordered by the
     *  probability (XXX) of each label being correct
     *  based on our current set of observations
     */
    public Classifications classify(double[] features)
    {
        // Find the k nearest neighbors
        // By sorting the list, we ensure that the first
        // k entries of the list of points are also the
        // nearest neighbors.
        Collections.sort(points, new CPointComparator(features));

        // Evaluate the neighbors based on the weights
        MultiGaussian mg = new MultiGaussian(P, features);
        HashMap<String, Double> labelWeights = new HashMap<String, Double>();
        double totalWeight = 0;

        for (int i = 0; i < Math.min(k, points.size()); i++) {
            CPoint point = points.get(i);
            double weight = mg.prob(point.coords);
            totalWeight += weight;
            double oldWeight = 0;
            if (labelWeights.containsKey(point.label)) {
               oldWeight = labelWeights.get(point.label);
            }
            labelWeights.put(point.label, oldWeight+weight);
        }

        // Compute the probability (XXX) of each label being
        // correct. Currently, these "probabilities" are
        // computed as fractions of the entire weight.
        Classifications classifications = new Classifications();
        for (String label: labelWeights.keySet()) {
            classifications.add(label, labelWeights.get(label)/totalWeight);
        }

        // Return
        return classifications;
    }

    /** Clear the classifier information */
    public void clearData()
    {
        points.clear();
    }

    /** Load in the classifier data */
    public void loadData()
    {
    }

    public static void main(String[] args)
    {
        // Test code!
    }
}
