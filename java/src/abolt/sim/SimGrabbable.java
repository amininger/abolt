package abolt.sim;

public interface SimGrabbable
{
    void setPos(double[] xyzrpy);
    public boolean inActionRange(double[] xyt);
}
