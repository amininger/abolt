package abolt.sim;

public interface SimGrabbable
{
    void setLoc(double[] xyt);
    public boolean inActionRange(double[] xyt);
}