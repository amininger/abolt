package abolt.arm;

import javax.swing.*;

import april.vis.*;

import abolt.lcmtypes.*;

/** Defines a generic interface for interacting with
 *  arm joints. Mainly exists to make interactions with
 *  joints clean and to make such tasks as rendering
 *  a long chain of arm segments a short, simple loop
 */
public interface Joint
{
    /** Render the joint in question */
    public VisObject getVis();

    /** Get rotation component */
    public double[][] getRotation();

    /** Get translation component */
    public double[][] getTranslation();

    /** Get the length of the associated arm segment */
    public double getLength();

    /** Get the actual position of the associated joint */
    public double getActualValue();

    /** Get the desired position of the associated joint */
    public double getDesiredValue();

    /** Get min range of motion */
    public double getMinValue();

    /** Get max range of motion */
    public double getMaxValue();

    /** Get an arm command back */
    public dynamixel_command_t getArmCommand();

    // ==================================

    /** Set the desired position to val */
    public void setPos(double val);

    /** Update the actual state of this joint */
    public void updatePos(double val);
}
