package abolt.util;

import java.util.*;

/** Bin n-D points based on the supplied index dimensions and resolution.
 *  e.g. take a bunch of 3D points and distribute them across XY.
 */
public class Binner
{
    static public ArrayList<double[]> binPoints(ArrayList<double[]> points, ArrayList<Integer> idxs, double r)
    {
        HashMap<Bin, double[]> binMap = new HashMap<Bin, double[]>();

        for (double[] p: points) {
            if (p == null) {
                continue;
            }
            Bin bin = new Bin(p, idxs, r);
            binMap.put(bin, p);
        }

        //System.out.printf("Shrank %d pts to %d\n", points.size(), binMap.size());
        ArrayList<double[]> newPoints = new ArrayList<double[]>();
        newPoints.addAll(binMap.values());

        return newPoints;
    }

    static public class Bin
    {
        double r;
        ArrayList<Integer> idxs;
        double[] p;

        public Bin(double[] p_, ArrayList<Integer> idxs_, double r_)
        {
            p = p_;
            idxs = idxs_;
            r = r_;
        }

        public int hashCode()
        {
            int code = 0;
            int[] b = bin();
            for (Integer i: b) {
                code ^= i.hashCode();
            }

            return code;
        }

        public boolean equals(Object o)
        {
            if (o == null)
                return false;
            if (!(o instanceof Bin))
                return false;
            Bin b = (Bin)o;
            return Arrays.equals(bin(), b.bin());
        }

        private int[] bin()
        {
            int[] vals = new int[idxs.size()];
            for (int i = 0; i < idxs.size(); i++) {
                int idx = idxs.get(i);
                vals[i] = (int)Math.floor(p[idx]/r);
            }

            return vals;
        }

    }
}
