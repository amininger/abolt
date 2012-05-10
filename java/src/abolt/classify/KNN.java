package abolt.classify;

import java.io.Console;
import java.lang.*;
import java.io.*;
import java.util.*;
import java.util.regex.Pattern;
import java.util.regex.Matcher;

import abolt.bolt.BoltObject;
import abolt.classify.Features.FeatureCategory;
import abolt.kinect.ObjectInfo;

/*
 * @author James Kirk
 */
class KNNPoint {
    List<Double> coords;
    String label;
    
    public KNNPoint(String featureString){
    	this.coords = FEUtil.getFeaturesFromString(featureString);
    	this.label = FEUtil.getLabelFromString(featureString);
    }
    
    public KNNPoint(double[] coords, String label) {
    	this.label = "unknown";
		this.coords = new ArrayList<Double>(coords.length);
		for (int i = 0; i < coords.length; i++)
		    this.coords.add(coords[i]);
	}

    public KNNPoint(List<Double> coords, String label) {
        this.coords = new ArrayList<Double>(coords.size());
        for (int i = 0; i < coords.size(); i++)
            this.coords.add(coords.get(i));
    }

    public KNNPoint(KNNPoint p) {
        this(p.getCoords(), p.getLabel());
    }
    
    public String getLabel(){
    	return label;
    }

    public List<Double> getCoords() {
        return coords;
    }

    public double distance(KNNPoint p) {
        List<Double> points2 = p.getCoords();
        double sum = 0;
        for (int i = 0; i < coords.size() && i < points2.size(); i++)
            sum += Math.pow(coords.get(i) - points2.get(i), 2);

        return Math.sqrt(sum);
    }
}

public class KNN implements IClassifier{
    protected List<KNNPoint> data;
    protected String datafile;

    protected int Km;
    protected int dim;

    protected double maxDistance;

    public KNN(int Km, int dim, String datafile, double maxDistance) {
        this.Km = Km;
        this.datafile = datafile;
        this.dim = dim;
        this.maxDistance = maxDistance;
        data = new ArrayList<KNNPoint>();
    }
    
    @Override
    public void add(ArrayList<Double> features, String label){
		KNNPoint pt = new KNNPoint(features, label);
		add(pt);
    }

    public void add(KNNPoint pt) {
    	if(pt.getCoords().size() != dim){
    		return;
    	}
        data.add(pt);
    }
    
	@Override
	public ConfidenceLabel classify(ArrayList<Double> features) {
		KNNPoint test = new KNNPoint(features, null);
		if(test.getCoords().size() != dim){
			// wrong number of dimensions
			return new ConfidenceLabel(0.0, "unknown");
		}

        List<ConfidenceLabel> cl = getMostConfidentLabels(this.Km, test);
        if (cl == null || cl.size() == 0) {
            return new ConfidenceLabel(0.0, "unknown");
        }
        ConfidenceLabel c = cl.get(0);
        return c;
	}


    public KNNPoint getNearestNeighbor(KNNPoint queryPt) {
    	if(queryPt.getCoords().size() != dim){
    		return null;
    	}
        double min = Double.MAX_VALUE;
        KNNPoint nearestPt = null;

        for (KNNPoint pt : data) {
            double dist = queryPt.distance(pt);
            if (dist < min && dist < maxDistance) {
                min = dist;
                nearestPt = pt;
            }
        }

        return nearestPt;
    }

    public List<KNNPoint> getKNearestNeighbors(int k, KNNPoint p) {
    	if(p.getCoords().size() != dim){
    		return null;
    	}
        List<KNNPoint> nearest = new ArrayList<KNNPoint>();

        for (KNNPoint ts : data) {
            double dist = ts.distance(p);
            if (nearest.size() < k) {
                if(dist<maxDistance)
                    nearest.add(ts);
                continue;
            }

            int replace = -1;

            // find the farthest point that is father than ts
            // if there is such a point and if so replace
            for (int i = 0; i < nearest.size(); i++) {
                KNNPoint cur = nearest.get(i);

                if (dist < cur.distance(p) && dist < maxDistance) {
                    replace = i;
                    dist = cur.distance(p);
                }
            }
            if (replace >= 0)
                nearest.set(replace, ts);
        }

        return nearest;
    }

    public List<ConfidenceLabel> getMostConfidentLabels(int k, KNNPoint p) {
    	if(p.getCoords().size() != dim){
    		return null;
    	}
        List<KNNPoint> nearest = getKNearestNeighbors(k, p);
        List<ConfidenceLabel> cl = new ArrayList<ConfidenceLabel>();

        if (nearest.isEmpty()) {
            return null;
        }

        List<String> answers = new ArrayList<String>();
        for (int i = 0; i < nearest.size(); i++) {
            KNNPoint ts = nearest.get(i);

            String label = ts.getLabel();
            if (answers.contains(label))
                continue;

            answers.add(label);

            int count = 1;
            for (int j = 0; j < nearest.size(); j++) {
                if (i == j) {
                    continue;
                }
                KNNPoint ts2 = nearest.get(j);

                if (label.equals(ts2.getLabel())) {
                    count++;
                }
            }

            cl.add(new ConfidenceLabel(((double) count)
                                       / ((double) nearest.size()), label));
        }
        Collections.sort(cl);
        return cl;
    }

    @Override
    public void clearData(){
        data = new ArrayList<KNNPoint>();
    }

    @Override
    public void loadData() {
        try {
            FileInputStream fstream = new FileInputStream(this.datafile);
            DataInputStream in = new DataInputStream(fstream);
            BufferedReader br = new BufferedReader(new InputStreamReader(in));
            String strLine;
            // Read File Line By Line
	    int i = 0;
	    while ((strLine = br.readLine()) != null) {
            add(new KNNPoint(strLine));
	    }
	    in.close();
        } catch (Exception e) {
            System.err.println("Error: " + e.getMessage());
        }
    }

    public double LOOCV() {
        int correct = 0;
        int total = 0;
        for (int i = 0; i < data.size(); i++) {
            KNNPoint out = data.get(i);
            String label = testSample(out);
            if (label.equals("unknown")) {
                continue;
            }
            if (label.equals(out.getLabel())) {
                correct++;
            }
            total++;
        }
        return (double) correct / (double) total;
    }

    private String testSample(KNNPoint testPoint) {
        List<KNNPoint> nearest = getKNearestNeighbors(this.Km + 1, testPoint);
        // remove self from testing
        if (!nearest.remove(testPoint)) {
            System.out.println("ERROR in Leave One out Cross Validation");
            return "unknown";
        }
        if (nearest.isEmpty()) {
            return "unknown";
        }

        List<ConfidenceLabel> cl = new ArrayList<ConfidenceLabel>();
        List<String> answers = new ArrayList<String>();

        for (int i = 0; i < nearest.size(); i++) {
            KNNPoint ts = nearest.get(i);

            String label = ts.getLabel();
            if (answers.contains(label))
                continue;

            answers.add(label);

            int count = 1;
            for (int j = 0; j < nearest.size(); j++) {
                if (i == j) {
                    continue;
                }
                KNNPoint ts2 = nearest.get(j);

                if (label.equals(ts2.getLabel())) {
                    count++;
                }
            }

            cl.add(new ConfidenceLabel(((double) count)
                                       / ((double) nearest.size()), label));
        }

        Collections.sort(cl);
        ConfidenceLabel best = cl.get(0);
        return best.getLabel();
    }
}
