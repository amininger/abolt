package abolt.collision;

import java.awt.*;
import java.io.*;

import april.vis.*;
import april.jmat.*;

public class SphereShape implements Shape
{
    protected double r;

    public SphereShape(double r)
    {
        this.r = r;
    }

    public double getRadius()
    {
        return r;
    }

    public double getBoundingRadius()
    {
        return r;
    }

	@Override
	public double collisionRay(double[] pos, double[] dir, double[][] T) {
		// unit vector from pos to center of sphere
        double xyz[] = new double[] { T[0][3], T[1][3], T[2][3] };
        double e[] = LinAlg.normalize(LinAlg.subtract(xyz, pos));

        // cosine of theta between dir and vector that would lead to
        // center of circle
        double costheta = LinAlg.dotProduct(e, dir);

        // law of cosines gives us:
        // r^2 = x^2 + d^2 - 2xd cos(theta)
        double d = LinAlg.distance(xyz, pos);

        // solve for x using quadratic formula
        double A = 1;
        double B = -2*d*costheta;
        double C = d*d - r*r;

        // no collision
        if (B*B - 4*A*C < 0)
            return Double.MAX_VALUE;

        double x1 = (-B - Math.sqrt(B*B - 4 * A * C)) / (2*A);
        if (x1 >= 0)
            return x1;

        double x2 = (-B + Math.sqrt(B*B - 4 * A * C)) / (2*A);
        if (x2 >= 0)
            return x2;

        return Double.MAX_VALUE;
	}
    
}
