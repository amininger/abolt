package abolt.classify;

import java.util.*;

/** A temporary hack to get shape rotation
 *  integrated into the system. Will die when
 *  shape classification is updated.
 */
public class ShapeGKNN extends GKNN
{
    public ShapeGKNN(int k_, double p_)
    {
        super(k_, p_);
    }

    public void add(ArrayList<Double> features, String label)
    {
        if (label == null)
            return;

        double first = features.get(0);
        ArrayList<Double> pt1 = new ArrayList<Double>();
        ArrayList<Double> pt2 = new ArrayList<Double>();

        double size = features.size();
        for(int i = 1; i < size; i++){
            if (i < (size/2)+1) {
                pt1.add(features.get(i));
            } else {
                pt2.add(features.get(i));
            }
        }

        // Regular
        features = new ArrayList<Double>();
        features.add(first);
        features.addAll(pt1);
        features.addAll(pt2);
        super.add(features, label);

        // Vertically Flipped
        features = new ArrayList<Double>();
        features.add(first);
        features.addAll(pt2);
        features.addAll(pt1);
        super.add(features, label);

        Collections.reverse(pt1);
        Collections.reverse(pt2);

        // Horizontally Flipped
        features = new ArrayList<Double>();
        features.add(first);
        features.addAll(pt1);
        features.addAll(pt2);
        super.add(features, label);

        // Horizontally and Vertically Flipped
        features = new ArrayList<Double>();
        features.add(first);
        features.addAll(pt2);
        features.addAll(pt1);
        super.add(features, label);
    }
}
