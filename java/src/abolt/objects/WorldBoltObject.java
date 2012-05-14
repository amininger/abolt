package abolt.objects;

import java.awt.Color;
import java.io.IOException;
import java.util.ArrayList;

import abolt.classify.ColorFeatureExtractor;
import abolt.classify.ConfidenceLabel;
import abolt.classify.Features;
import abolt.classify.Features.FeatureCategory;
import abolt.classify.SizeFeatureExtractor;
import abolt.kinect.ObjectInfo;
import april.jmat.LinAlg;
import april.sim.Shape;
import april.sim.SphereShape;
import april.util.StructureReader;
import april.util.StructureWriter;
import april.vis.Style;
import april.vis.VisChain;
import april.vis.VisObject;
import april.vis.VzBox;
import april.vis.VzMesh;

public class WorldBoltObject extends BoltObject {
	private ObjectInfo info;
	private SphereShape shape;
	private VisChain model;
	private VzBox vzBox;

	public WorldBoltObject(int id) {
		super(id);
		shape = new SphereShape(.01);
        model = new VisChain(LinAlg.scale(.01),
                new VzBox(new VzMesh.Style(Color.white)));
	}

	@Override
	public ArrayList<Double> getFeatures(FeatureCategory cat) {
		return info.getFeatures(cat);
	}

	@Override
	public Shape getShape() {
		return shape;
	}

	@Override
	public VisObject getVisObject() {
		return model;
	}
	
	public ObjectInfo getInfo(){
		return info;
	}
	
	public void updateObject(ObjectInfo info){
		this.info = info;
		double[] bb = SizeFeatureExtractor.boundingBoxWorld(info.points);
        double[] min = new double[]{bb[0], bb[1], bb[2]};
        double[] max = new double[]{bb[3], bb[4], bb[5]};
        double[] xyzrpy = new double[]{0, 0, 0, 0, 0, 0};
        double maxDim = 0;
        for(int i = 0; i < 3; i++){
            xyzrpy[i] = (min[i] + max[i])/2;
            if(max[i] - min[i] > maxDim){
            	maxDim = max[i] - min[i];
            }
        }
        maxDim /= 2;
        double[] center = new double[]{xyzrpy[0], xyzrpy[1], xyzrpy[2]};

        LinAlg.minusEquals(min, center);
        LinAlg.minusEquals(max, center);
        
        shape = new SphereShape(maxDim);
        Color color = ColorFeatureExtractor.getColorFromFeatures(info.getFeatures(FeatureCategory.COLOR));
        model = new VisChain(LinAlg.translate(center), LinAlg.scale(max[0] - min[0], max[1] - min[1], max[2] - min[2]), 
                new VzBox(new VzMesh.Style(color)));
        bbox[0] = min;
        bbox[1] = max;
        pos = xyzrpy;
	}



    public void read(StructureReader ins) throws IOException
    {
    	// This is designed to only be created from camera data, 
    	// not from reading from files
    }

    public void write(StructureWriter outs) throws IOException
    {
    	outs.writeComment("XYZRPY Truth");
        outs.writeDoubles(LinAlg.matrixToXyzrpy(getPose()));
        for(FeatureCategory cat : FeatureCategory.values()){
        	ConfidenceLabel label = labels.getBestLabel(cat);
        	outs.writeString(label.getLabel() + "(" + label.getConfidence() + ")");
        }
    }

    public void setRunning(boolean run)
    {

    }

    public boolean inSenseRange(double[] xyt)
    {
    	return true;
    }
}
