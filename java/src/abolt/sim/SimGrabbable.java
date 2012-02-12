package abolt.sim;

public interface SimGrabbable
{
    void setLoc(double[] xyzrpy);
    public boolean inActionRange(double[] xyt);
}
