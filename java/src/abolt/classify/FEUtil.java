package abolt.classify;

import java.util.ArrayList;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

/**
 * @author Aaron 
 * @purpose Contains utility functions used by the FeatureExtractors
 */
public abstract class FEUtil {
	protected static void divideEquals(double[] values, double divisor)
    {
		assert (divisor != 0);
		for (int i = 0; i < values.length; i++) {
			values[i] /= divisor;
		}
	}

	protected static void addArray(ArrayList<Double> list, double[] additions)
    {
		for (double d : additions) {
			list.add(d);
		}
	}
	
	public static String featuresToString(ArrayList<Double> features){
		if(features == null || features.size() == 0){
			return "[]";
		}
		String s = "[";
		for (Double d : features) {
			s += d + " ";
		}
		s += "]";
		return s;
	}
	
	public static String getLabelFromString(String featureString){
		String reg = "\\{.*\\}";
        Pattern p = Pattern.compile(reg);

        Matcher m = p.matcher(featureString);
        String label;
        if (m.find()) {
            label = m.group();
            label = label.replace("{", "");
            label = label.replace("}", "");
            return label;
        } else {
            System.out.println("ERROR: no label found on training data");
            return "unknown";
        }
	}
	
	public static ArrayList<Double> getFeaturesFromString(String featureString){
		String reg = "[0-9]{1,}+(\\.[0-9]{1,})";
        Pattern p = Pattern.compile(reg);

        Matcher m = p.matcher(featureString);

        ArrayList<Double> features = new ArrayList<Double>();
        while (m.find()) {
        	Double val = Double.valueOf(m.group());
        	if(val != null){
        		features.add(val);
        	}
        }
        return features;
	}
}
