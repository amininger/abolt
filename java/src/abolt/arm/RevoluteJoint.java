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

    public VisObject getVis()
    {
        return vobj;
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
        public double dAngle = 0.0;
        public double aAngle = 0.0;
        public double lSegment = 0.0;
        public double rMin = -Math.PI;
        public double rMax = Math.PI;
        public double[] orientation = Z_AXIS;

        // Servo settings
        public double speed = 0.1;
        public double torque = 0.6;
    }

    /** Set the joint to the requested angle */
    public void setPos(double val)
    {
        params.dAngle = MathUtil.clamp(val, params.rMin, params.rMax);
    }

    /** Take an actual update value and track the real position
     *  of this servo
     */
    public void updatePos(double val)
    {
        params.aAngle = val;
        rotation = LinAlg.quatToMatrix(LinAlg.angleAxisToQuat(params.aAngle, params.orientation));
    }

    /** Get the arm command for the current position */
    public dynamixel_command_t getArmCommand()
    {
        dynamixel_command_t cmd = new dynamixel_command_t();
        cmd.position_radians = MathUtil.mod2pi(MathUtil.clamp(params.dAngle, params.rMin, params.rMax));
        cmd.speed = params.speed;
        cmd.max_torque = params.torque;

        // XXX Receiver sets utimes
        return cmd;

    }

    // ==============
    public double getLength()
    {
        return params.lSegment;
    }

    public double getActualValue()
    {
        return params.aAngle;
    }

    public double getDesiredValue()
    {
        return params.dAngle;
    }

    public double getMinValue()
    {
        return params.rMin;
    }

    public double getMaxValue()
    {
        return params.rMax;
    }

}
