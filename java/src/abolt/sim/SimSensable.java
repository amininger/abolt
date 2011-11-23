package abolt.sim;

public interface SimSensable
{
    public String getName();
    public String getProperties();
    public boolean inRange(double[] xyt);
}
