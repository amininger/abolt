package abolt.classify;

import java.util.*;
import java.io.*;

import april.jmat.*;
import april.util.*;

import abolt.util.*;

/** K-nearest neighbor classifier using a guassian-like
 *  weighting function when considering the importance
 *  of the neighbors
 */
public class GKNN implements IClassifier
{
    ArrayList<CPoint> points = new ArrayList<CPoint>();

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
    double p;

    // Load data from this file
    String filename = null;

    /** Initialize the KNN
     *  @param k_       The max number of neighbors considered
     *  @param p_       A covariance matrix parameter affecting weighting
     */
    public GKNN(int k_, double p_)
    {
        k = k_;
        p = p_;
    }


    /** Add a training example to the classifier */
    synchronized public void add(ArrayList<Double> features, String label)
    {
        if (label == null)
            return;
        double[] dfeatures = BoltMath.toArray(features, new double[features.size()]);
        add(dfeatures, label);
    }

    public void add(double[] features, String label)
    {
        if (label == null)
            return;
        points.add(new CPoint(label, features));
    }

    synchronized public void add(CPoint point)
    {
        if (point == null)
            return;
        points.add(point);
    }

    @Override
    synchronized public CPoint removeLast()
    {
        if (points.size() > 0) {
            CPoint last = points.remove(points.size()-1);
            return last;
        }

        return null;
    }

    @Override
    synchronized public Classifications classify(ArrayList<Double> features)
    {
        return classify(BoltMath.toArray(features, new double[features.size()]));
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
        ArrayList<CPoint> sortedPoints = (ArrayList<CPoint>)points.clone();
        Collections.sort(sortedPoints, new CPointComparator(features));

        // Evaluate the neighbors based on the weights
        double[][] P = LinAlg.scale(LinAlg.identity(features.length), p);
        MultiGaussian mg = new MultiGaussian(P, features);
        double maxValue = mg.prob(features);

        HashMap<String, Double> labelWeights = new HashMap<String, Double>();
        HashMap<String, Integer> labelExamples = new HashMap<String, Integer>();
        double totalWeight = 0;

        for (int i = 0; i < Math.min(k, sortedPoints.size()); i++) {
            CPoint point = sortedPoints.get(i);
            double weight = mg.prob(point.coords);
            totalWeight += weight;
            double oldWeight = 0;
            int numExamples = 0;
            if (labelWeights.containsKey(point.label)) {
               oldWeight = labelWeights.get(point.label);
               numExamples = labelExamples.get(point.label);
            }
            labelWeights.put(point.label, oldWeight+weight);
            labelExamples.put(point.label, numExamples+1);
        }

        // Compute the probability (XXX) of each label being
        // correct. Currently, these "probabilities" are
        // computed as fractions of the entire weight.
        Classifications classifications = new Classifications();
        for (String label: labelWeights.keySet()) {
            double labelWeight = labelWeights.get(label);

            // Total weight normalizer
            double weightNormalizer = labelWeight/totalWeight;

            // Weight normalization
            labelWeight /= labelExamples.get(label);
            labelWeight /= maxValue;
            labelWeight *= weightNormalizer;
            classifications.add(label, labelWeight);
        }

        // If we have no label, report the unknown label, for now
        if (classifications.size() < 1) {
            classifications.add("unknown", 0.0);
        }
        return classifications;
    }

    /** Clear the classifier information */
    synchronized public void clearData()
    {
        points.clear();
    }

    /** Load in the classifier data from file*/
    synchronized public void loadData()
    {
        if (filename == null)
            return;

        try {
            Scanner scanner = new Scanner(new File(filename));
            scanner.useDelimiter("[\\s\\[\\]\\{\\}]+");

            while (scanner.hasNext()) {
                ArrayList<Double> features = new ArrayList<Double>();
                while (scanner.hasNextDouble()) {
                    features.add(scanner.nextDouble());
                }
                String label = scanner.next();
                add(features, label);
            }
        } catch (IOException ioex) {
            System.err.println("ERR: Trouble loading GKNN file");
            ioex.printStackTrace();
        }

        //LOOCV();
    }

    /** Set the file to load training data from on a load call */
    public void setDataFile(String filename_)
    {
        filename = filename_;
    }

    // Leave one out cross validation
    public void LOOCV()
    {
        HashMap<String, Pair<Integer, Integer> > accumulator = new HashMap<String, Pair<Integer, Integer> >();
        int size = points.size();
        for (int i = 0; i < size; i++) {
            // Take out a test point
            ArrayList<CPoint> training = new ArrayList<CPoint>(points);
            CPoint cp = training.remove(i);

            // Swap the training points - 1 in for classification
            ArrayList<CPoint> temp = points;
            points = training;

            if (!accumulator.containsKey(cp.label)) {
                accumulator.put(cp.label, new Pair<Integer, Integer>(0, 0));
            }
            Pair<Integer, Integer> pair = accumulator.get(cp.label);
            pair.o1 += 1;

            Classifications cs = classify(cp.coords);
            String label = cs.getBestLabel().label;

            if (cp.label.equals(label)) {
                pair.o2 += 1;
            }

            accumulator.put(cp.label, pair);

            // Swap back in the old points
            points = temp;
        }

        // Print out stats
        System.out.println("=============");
        for (String label: accumulator.keySet()) {
            Pair<Integer, Integer> pair = accumulator.get(label);
            System.out.printf("%s: %f\n", label, (double)pair.o2/(double)pair.o1);
        }
    }

    // =============================================

    public static void main(String[] args)
    {
        // Test code!
        GKNN knn = new GKNN(10, 0.1);

        // Fake features to add
        double[] red0 = new double[] {1.0, 0.1, 0.2};
        double[] red1 = new double[] {0.75, 0.3, 0.3};
        double[] red2 = new double[] {0.8, 0.3, 0.35};
        double[] red3 = new double[] {0.9, 0.23, 0.3};
        double[] red4 = new double[] {1.0, 0, 0};
        double[] red5 = new double[] {0.94, 0.15, 0.18};
        double[] red6 = new double[] {0.95, 0.75, 0.73};
        double[] red7 = new double[] {0.49, 0.03, 0.04};
        double[] red8 = new double[] {0.38, 0.1, 0.04};
        double[] red9 = new double[] {0.61, 0.2, 0.32};

        double[] green0 = new double[] {0.3, 0.8, 0.4};
        double[] green1 = new double[] {0.1, 0.4, 0};
        double[] green2 = new double[] {0.2, 0.98, 0.3};
        double[] green3 = new double[] {0.13, 0.82, 0.1};
        double[] green4 = new double[] {0.23, 0.79, 0.07};

        double[] blue0 = new double[] {0.2, 0.1, 0.9};
        double[] blue1 = new double[] {0.1, 0.3, 0.8};
        double[] blue2 = new double[] {0.3, 0.21, 0.83};
        double[] blue3 = new double[] {0.03, 0.05, 1.0};
        double[] blue4 = new double[] {0.45, 0.5, 0.98};
        double[] blue5 = new double[] {0.02, 0.03, 0.4};
        double[] blue6 = new double[] {0.14, 0.12, 0.79};
        double[] blue7 = new double[] {0.21, 0.24, 0.91};

        double[] purple0 = new double[] {0.4, 0.1, 0.5};
        double[] purple1 = new double[] {0.8, 0.3, 0.9};
        double[] purple2 = new double[] {0.59, 0.2, 0.7};
        double[] purple3 = new double[] {0.78, 0.5, 0.9};
        double[] purple4 = new double[] {0.39, 0.04, 0.4};
        double[] purple5 = new double[] {0.66, 0.21, 0.84};

        int r = 0, g = 0, b = 0, p = 0;
        r+=1;
        printRGBP(r,g,b,p);
        knn.add(red0, "red");
        test(knn);

        g+=1;
        printRGBP(r,g,b,p);
        knn.add(green0, "green");
        test(knn);

        b+=1;
        printRGBP(r,g,b,p);
        knn.add(blue0, "blue");
        test(knn);

        r+=1;
        printRGBP(r,g,b,p);
        knn.add(red1, "red");
        test(knn);

        r+=3;
        printRGBP(r,g,b,p);
        knn.add(red2, "red");
        knn.add(red3, "red");
        knn.add(red4, "red");
        test(knn);

        r+=5;
        printRGBP(r,g,b,p);
        knn.add(red5, "red");
        knn.add(red6, "red");
        knn.add(red7, "red");
        knn.add(red8, "red");
        knn.add(red9, "red");
        test(knn);

        g+=4;
        printRGBP(r,g,b,p);
        knn.add(green1, "green");
        knn.add(green2, "green");
        knn.add(green3, "green");
        knn.add(green4, "green");
        test(knn);

        b+=6;
        printRGBP(r,g,b,p);
        knn.add(blue1, "blue");
        knn.add(blue2, "blue");
        knn.add(blue3, "blue");
        knn.add(blue4, "blue");
        knn.add(blue5, "blue");
        knn.add(blue6, "blue");
        test(knn);

        p+=6;
        printRGBP(r,g,b,p);
        knn.add(purple0, "purple");
        knn.add(purple1, "purple");
        knn.add(purple2, "purple");
        knn.add(purple3, "purple");
        knn.add(purple4, "purple");
        knn.add(purple5, "purple");
        test(knn);
    }

    static private void printRGBP(int r, int g, int b, int p)
    {
        System.out.printf("\nr: [%d]\tg: [%d]\tb: [%d]\tp: [%d]\n========================\n", r,g,b,p);
    }

    static private void test(GKNN knn)
    {
        // Test colors
        double[] red = new double[] {0.63, 0.32, 0.29};
        double[] green = new double[] {0.2, 0.8, 0.3};
        double[] blue = new double[] {0.4, 0.3, 0.89};
        double[] purple = new double[] {0.7, 0.2, 0.9};
        System.out.printf("Classify red: %s\n\n", knn.classify(red));
        System.out.printf("Classify green: %s\n\n", knn.classify(green));
        System.out.printf("Classify blue: %s\n\n", knn.classify(blue));
        System.out.printf("Classify purple: %s\n\n", knn.classify(purple));
    }
}
