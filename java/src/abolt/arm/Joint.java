package abolt.arm;

import javax.swing.*;

import april.vis.*;

import abolt.lcmtypes.*;

/** All joint types define an input panel for selecting
 *  their parameters as well as for solving the system.
 *  Joints are automatically designated as having an
 *  associated segment, of which the joint is the base.
 *  If this segment is zero, it merely affects the
 *  orientation of the next joint.
 */
interface Joint
{
    /** Return a panel containing any knobs this
     *  joint wishes to expose.
     */
    public JPanel getParameterPanel(int n);

    /** Render the joint in question */
    public VisObject getVis();

    /** Get rotation component */
    public double[][] getRotation();

    /** Get translation component */
    public double[][] getTranslation();

    /** Set the movement value to val */
    public void set(double val);

    /** Get an arm command back */
    public dynamixel_command_t getArmCommand();
}
