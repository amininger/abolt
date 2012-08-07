package abolt.util;

import java.util.*;

public class BoltMath
{
    public static boolean equals(double a, double b, double thresh)
    {
        return Math.abs(a-b) < thresh;
    }

    public static double[] toArray(Collection<Double> collection, double[] da)
    {
        if (da == null || da.length < collection.size()) {
            da = new double[collection.size()];
        }
        int i = 0;
        for (Double d: collection) {
            da[i++] = d;
        }

        return da;
    }

    public static int[] toArray(Collection<Integer> collection, int[] ia)
    {
        if (ia == null || ia.length < collection.size()) {
            ia = new int[collection.size()];
        }
        int j = 0;
        for (Integer i: collection) {
            ia[j++] = i;
        }

        return ia;
    }
}
