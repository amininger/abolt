package abolt.util;

public class BoltMath
{
    public static boolean equals(double a, double b, double thresh)
    {
        return Math.abs(a-b) < thresh;
    }
}
