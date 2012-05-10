package abolt.kinect;

import java.util.*;

/** Container class for training data read in from .pts files */
public class LabeledPointCloud
{
    public ArrayList<String> labels;
    public ArrayList<double[]> coloredPoints;

    public LabeledPointCloud(ArrayList<String> labels_, ArrayList<double[]> coloredPoints_)
    {
        labels = labels_;
        coloredPoints = coloredPoints_;

        /*for (int i = 0; i < coloredPoints.size(); i++) {
            int rgb = (int)(coloredPoints.get(i)[3]);
            System.out.printf("rgb: %x\n", rgb);
        }*/
    }
}
