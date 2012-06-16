package abolt.classify;

import java.util.*;

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
        public int compareTo(Label l)
        {
            if (weight < l.weight)
                return 1;
            else if (weight > l.weight)
                return -1;
            return 0;
        }

        public String toString()
        {
            Formatter f = new Formatter();
            f.format("%s -- [%f]", label, weight);
            return f.toString();
        }
    }

    // All of the possible labels
    boolean sorted = false;
    public ArrayList<Label> labels;

    public Classifications()
    {
        labels = new ArrayList<Label>();
    }

    public void sortLabels()
    {
        Collections.sort(labels);
        sorted = true;
    }

    public void add(String label, double weight)
    {
        sorted = false;
        labels.add(new Label(label, weight));
    }

    public int size()
    {
        return labels.size();
    }

    public Label getBestLabel()
    {
        if (labels.size() < 1) {
            return null;    // XXX Return a null label type?
        }

        if (!sorted) {
            sortLabels();
        }

        return labels.get(0);
    }

    public String toString()
    {
        if (!sorted) {
            sortLabels();
        }

        StringBuilder sb = new StringBuilder();
        for (Label label: labels) {
            sb.append("\n"+label.toString());
        }

        return sb.toString();
    }

}
