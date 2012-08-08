package abolt.kinect;

import april.jmat.LinAlg;
import april.vis.VisCameraManager.CameraPosition;

public interface IBoltCamera {
	int[] getPixel(double[] cameraPt);
	double[] getWorldCoords(double[] cameraPt);
    double getHeight(double[] xy);
}
