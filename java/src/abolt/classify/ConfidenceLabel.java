package abolt.classify;

/**
 * @author aaron
 * @purpose A visual label/confidence pair produced by classification
 */
public class ConfidenceLabel implements Comparable<ConfidenceLabel> {
	double confidence;
	String label;

	public ConfidenceLabel(double confidence, String label) {
		this.confidence = confidence;
		this.label = label;
	}

	public double getConfidence() {
		return confidence;
	}

	public String getLabel() {
		return label;
	}

	@Override
	public int compareTo(ConfidenceLabel cl) {
		return new Double(this.confidence).compareTo(cl.confidence);
	}
}