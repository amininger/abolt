package abolt.arm;

import java.awt.*;
import javax.swing.*;

import april.jmat.*;
import april.util.*;
import april.vis.*;

import abolt.lcmtypes.*;

class RevoluteJoint implements Joint
{
    // Current VisObject
    VisObject vobj;

    // State
    double[][] rotation;
    double[][] translation;

    Parameters params;

    static public final double[] X_AXIS = new double[] {1,0,0};
    static public final double[] Y_AXIS = new double[] {0,1,0};
    static public final double[] Z_AXIS = new double[] {0,0,1};

    /** Initialize using default parameters */
    public RevoluteJoint()
    {
        this(new Parameters());
    }

    /** Initialize using specified parameters */
    public RevoluteJoint(Parameters params_)
    {
        params = params_;
        updateVisObject();

        rotation = LinAlg.rotateZ(0);
        translation = LinAlg.translate(0,0,params.lSegment);
    }

    public JPanel getParameterPanel(int n)
    {
        JPanel panel = new JPanel();
        panel.setBorder(BorderFactory.createTitledBorder("Revolute Joint "+n));
        ParameterGUI pg = new ParameterGUI();

        pg.addDoubleSlider("lSeg",
                           "Segment Length [m]",
                           0,
                           JointConstraints.MAX_L_SEG,
                           params.lSegment);
        pg.addDoubleSlider("angle",
                           "Angle [rad]",
                           JointConstraints.R_MIN,
                           JointConstraints.R_MAX,
                           0);
        pg.addDoubleSlider("rMin",
                           "Min Angle [rad]",
                           JointConstraints.R_MIN,
                           JointConstraints.R_MAX,
                           params.rMin);
        pg.addDoubleSlider("rMax",
                           "Max Angle [rad]",
                           JointConstraints.R_MIN,
                           JointConstraints.R_MAX,
                           params.rMax);
        pg.addCheckBoxes("X", "X", false,
                         "Y", "Y", false,
                         "Z", "Z", true);
        pg.addListener(new RevoluteListener());

        panel.add(pg);
        return panel;
    }

    public VisObject getVis()
    {
        return vobj;
    }

    public double[] getJacobian(double[][] preXform, double[][] postXform)
    {
        double[] J = new double[3];

        // Numerically compute the Jacobian of this joint. If moving
        // this joint would force it outside of the possible constraints,
        // then set derivatives to 0. (XXX)
        double dh = (Math.PI/3)*.1; // One tenth of a radian
        double a0 = params.angle - dh;
        double a1 = params.angle + dh;
        //if (a0 < params.rMin || a1 > params.rMax)
        //    return J;

        double[][] r0 = LinAlg.quatToMatrix(LinAlg.angleAxisToQuat(a0, params.orientation));
        double[][] r1 = LinAlg.quatToMatrix(LinAlg.angleAxisToQuat(a1, params.orientation));

        double[] xyzrpy0, xyzrpy1;
        double[][] xform0 = LinAlg.matrixAB(preXform, r0);
        LinAlg.timesEquals(xform0, translation);
        LinAlg.timesEquals(xform0, postXform);
        double[][] xform1 = LinAlg.matrixAB(preXform, r1);
        LinAlg.timesEquals(xform1, translation);
        LinAlg.timesEquals(xform1, postXform);
        xyzrpy0 = LinAlg.matrixToXyzrpy(xform0);
        xyzrpy1 = LinAlg.matrixToXyzrpy(xform1);

        J[0] = (xyzrpy1[0] - xyzrpy0[0])/dh;
        J[1] = (xyzrpy1[1] - xyzrpy0[1])/dh;
        J[2] = (xyzrpy1[2] - xyzrpy0[2])/dh;

        return J;
    }

    public double[][] getRotation()
    {
        return rotation;
    }

    public double[][] getTranslation()
    {
        return translation;
    }

    // ======================
    private void updateVisObject()
    {
        double ch = 0.03;
        double cr = 0.01;
        double bs = 0.01;
        VzCylinder cyl = new VzCylinder(cr, ch, new VzMesh.Style(Color.red));
        double[][] xForm = LinAlg.identity(4);
        if (!LinAlg.equals(Z_AXIS, LinAlg.normalize(params.orientation), 0.00001)) {
            double[] q = LinAlg.quatCompute(Z_AXIS, LinAlg.normalize(params.orientation));
            xForm = LinAlg.quatToMatrix(q);
        }
        VisChain rCyl = new VisChain(xForm, cyl);
        if (params.lSegment > 0) {
            VzBox box = new VzBox(bs, bs, params.lSegment, new VzMesh.Style(Color.blue));
            vobj = new VisChain(rCyl,
                                new VisChain(LinAlg.translate(0, 0, params.lSegment/2),
                                             box));
        } else {
            vobj = rCyl;
        }
    }


    static public class Parameters
    {
        public double angle = 0.0;
        public double lSegment = 0.0;
        public double rMin = JointConstraints.R_MIN;
        public double rMax = JointConstraints.R_MAX;
        public double[] orientation = Z_AXIS;
    }

    class RevoluteListener implements ParameterListener
    {
        public void parameterChanged(ParameterGUI pg, String name)
        {
            synchronized (params) {
                if (name.equals("lSeg")) {
                    params.lSegment = pg.gd("lSeg");
                    translation = LinAlg.translate(0,0,params.lSegment);
                } else if (name.equals("rMin")) {
                    params.rMin = pg.gd("rMin");
                    params.rMax = Math.max(params.rMax, params.rMin);
                    params.angle = MathUtil.clamp(params.angle, params.rMin, params.rMax);
                    pg.sd("rMax", params.rMax);
                    pg.sd("angle", params.angle);
                } else if (name.equals("rMax")) {
                    params.rMax = pg.gd("rMax");
                    params.rMin = Math.min(params.rMin, params.rMax);
                    params.angle = MathUtil.clamp(params.angle, params.rMin, params.rMax);
                    pg.sd("rMin", params.rMin);
                    pg.sd("angle", params.angle);
                } else if (name.equals("X")) {
                    pg.sb("X", true);
                    pg.sb("Y", false);
                    pg.sb("Z", false);
                    params.orientation = X_AXIS;
                } else if (name.equals("Y")) {
                    pg.sb("X", false);
                    pg.sb("Y", true);
                    pg.sb("Z", false);
                    params.orientation = Y_AXIS;
                } else if (name.equals("Z")) {
                    pg.sb("X", false);
                    pg.sb("Y", false);
                    pg.sb("Z", true);
                    params.orientation = Z_AXIS;
                } else if (name.equals("angle")) {
                    params.angle = MathUtil.clamp(pg.gd("angle"), params.rMin, params.rMax);
                    pg.sd("angle", params.angle);
                }

                rotation = LinAlg.quatToMatrix(LinAlg.angleAxisToQuat(params.angle, params.orientation));
                updateVisObject();
            }
        }
    }

    /** Sets the joint angle to angle+val*/
    public void delta(double val)
    {
        params.angle = MathUtil.clamp(params.angle+val, params.rMin, params.rMax);
        rotation = LinAlg.quatToMatrix(LinAlg.angleAxisToQuat(params.angle, params.orientation));
    }

    /** Set the joint to the requested angle */
    public void set(double val)
    {
        params.angle = MathUtil.clamp(val, params.rMin, params.rMax);
        rotation = LinAlg.quatToMatrix(LinAlg.angleAxisToQuat(params.angle, params.orientation));
    }

    /** Get the arm command for the current position */
    public dynamixel_command_t getArmCommand()
    {
        dynamixel_command_t cmd = new dynamixel_command_t();
        cmd.position_radians = MathUtil.mod2pi(MathUtil.clamp(params.angle, params.rMin, params.rMax));
        cmd.speed = 0.25;
        cmd.max_torque = 0.6;

        // XXX Receiver sets utimes
        return cmd;

    }

}
