package abolt.kinect;

public interface IBoltCamera {
	int[] getPixel(double[] cameraPt);
	double[] getWorldCoords(double[] cameraPt);
}
