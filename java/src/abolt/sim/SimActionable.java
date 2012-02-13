package abolt.sim;

public interface SimActionable
{
    public String[] getAllowedStates();
    public String getState();
    public void setState(String newState);
    //public boolean inActionRange(double[] xyt);
}
