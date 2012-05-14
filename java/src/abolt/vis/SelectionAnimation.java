package abolt.vis;

import java.awt.*;

import april.jmat.*;
import april.vis.*;

public class SelectionAnimation implements VisObject
{
    double[] xyz;
    double outerRad;

    // Animation
    static final double INNER_RAD = 0.01;   // [m]
    static final double MAX_RAD = .3;
    static final double TARGET_TIME = 1;    // [s]
    double SPEED;   // [m/s]

    // State
    double rad = INNER_RAD;

    public SelectionAnimation(double[] xyz_, double outerRad_)
    {
        xyz = xyz_;
        xyz[2] = 0.01;
        outerRad = outerRad_;
        if(outerRad > MAX_RAD){
        	outerRad = MAX_RAD;
        }

        SPEED = (outerRad - INNER_RAD)/TARGET_TIME;
    }

    /** Take a step forward dt seconds in the animation */
    public void step(double dt)
    {
        rad += dt*SPEED;
        if (rad > outerRad)
            rad = INNER_RAD;
    }

    public void render(VisCanvas vc, VisLayer vl, VisCanvas.RenderInfo rinfo, GL gl)
    {
        VzCircle circle = new VzCircle(rad, new VzLines.Style(Color.blue, 2));
        VisChain vch = new VisChain(LinAlg.translate(xyz),
                                    circle);
        vch.render(vc, vl, rinfo, gl);
    }
}
