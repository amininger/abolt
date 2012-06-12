package abolt.classify;

import java.util.;

public class Classifications
{
    static public class Label implements Comparable<Label>
    {
        public String label;
        public double weight;

        public Label(String label_, double weight_)
        {
            label = label_;
            weight = weight_;
        }

        // XXX Want an ascending sort
        public boolean compare(Label l)
        {
            if (weight < l.weight)
                return 1;
            else if (weight > l.weight)
                return -1;
            return 0;
        }
    }

    // All of the possible labels
    boolean sorted = false;
    public ArrayList<Label> labels;

    public Classifications()
    {
        label = new ArrayList<Label>();
    }

    public void add(String label, double weight)
    {
        sorted = false;
        labels.add(new Label(label, weight));
    }

    public Label getBestLabel()
    {
        if (labels.size() < 1) {
            return null;    // XXX Return a null label type?
        }

        if (!sorted) {
            Collections.sort(labels);
            sorted = true;
        }

        return labels.get(0);
    }

}
