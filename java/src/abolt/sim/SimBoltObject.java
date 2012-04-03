package abolt.sim;

import abolt.lcmtypes.categorized_data_t;
import april.sim.*;

public interface SimBoltObject
{
	public categorized_data_t[] getCategorizedData();
    public int getID();
    public double[][] getBBox();
}
