package abolt.classify;

import java.util.*;

import april.jmat.*;

import abolt.util.*;

/** An entry-coordinate pair for classification,
 */
public class CPoint
{
    public String label;
    public double[] coords;

    public CPoint(String label_, ArrayList<Double> coords_)
    {
        this(label_, BoltMath.toArray(coords_, new double[coords_.size()]));
    }

    public CPoint(String label_, double[] coords_)
    {
        label = label_;
        coords = coords_;
    }
}
