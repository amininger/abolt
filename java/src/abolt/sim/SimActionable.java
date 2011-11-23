package abolt.sim;

public interface SimActionable
{
    String[] getAllowedStates();
   
    String getState();

    void setState(String newState);

}