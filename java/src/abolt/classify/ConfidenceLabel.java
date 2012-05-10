package abolt.classify;

public class ConfidenceLabel implements Comparable<ConfidenceLabel>
{
    double confidence;
    String label;
    
    public ConfidenceLabel(double confidence, String label)
    {
	this.confidence = confidence;
	this.label = label;
    }
    
    public double getConfidence()
    {
	return confidence;
    }
    public String getLabel()
    {
    	return label;
    }
    
    @Override
    public int compareTo(ConfidenceLabel cl)
    {
	//ConfidenceLabel cl = (ConfidenceLabel) o;
	double diff = this.confidence - cl.confidence;
	if (diff < 0)
	    return (1);
	else if (diff > 0)
	    return (-1);
	else
	    return 0;
    }
    
}