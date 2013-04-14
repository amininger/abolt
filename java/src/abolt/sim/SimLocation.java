package abolt.sim;

import java.awt.Color;
import java.io.IOException;
import java.util.ArrayList;
import april.jmat.LinAlg;
import april.sim.BoxShape;
import april.sim.SimWorld;
import april.util.StructureReader;
import april.util.StructureWriter;
import april.vis.VisChain;
import april.vis.VisObject;
import april.vis.VzMesh;
import april.vis.VzRectangle;
import april.vis.VzText;

import abolt.util.SimUtil;

public class SimLocation extends SimBoltObject implements SimActionable {
    protected String name;

    protected double size = .1;
    protected SensableStates sensStates;
    protected abolt.collision.Shape aboltShape = null;

    public SimLocation(SimWorld sw)
    {
    	id = SimUtil.nextID();
    }
	
	public String getName() {
		return name;
	}
	
	public void setID(int id){}

    public VisObject getVisObject()
    {
        return constructModel();
    }
    
    public abolt.collision.Shape getAboltShape(){
		return aboltShape;
	}
    
    public double[][] getBBox(){
    	return new double[][]{
    			new double[]{-size, -size, -size}, 
    			new double[]{size, size, size}};
    }

	/*** Methods for SimActionable ***/
	public SensableStates getStates() {
		return sensStates;
	}
	
	public void setState(String keyValString) {
		sensStates.setState(keyValString);
	}

	public void read(StructureReader ins) throws IOException
    {
		double[] xy = ins.readDoubles();
    	pose = new double[]{xy[0], xy[1], 0, 0, 0, 0};

    	name = ins.readString();
    	
    	int[] colors = ins.readInts();
    	color = new Color(colors[0], colors[1], colors[2]);
    	
    	size = ins.readFloat();
        
        int numProps = ins.readInt();
        String[] props = new String[numProps];
        for(int i = 0; i < numProps; i++){
        	props[i] = ins.readString();
        }	
        sensStates = new SensableStates(props);
        sensStates.addStateSet("name", new String[]{name});

        shape = new BoxShape(new double[]{2*size, 2*size, 0});
        aboltShape = new abolt.collision.BoxShape(2*size, 2*size, .02);
    }
	

    public void write(StructureWriter outs) throws IOException
    {
    	outs.writeComment("XY");
        outs.writeDoubles(new double[]{pose[0], pose[1]});
        outs.writeInts(new int[]{color.getRed(), color.getGreen(), color.getBlue()});
    }
	
	private VisObject constructModel(){
		ArrayList<Object> objs = new ArrayList<Object>();

        // The larger box making up the background of the object
		objs.add(new VisChain(LinAlg.translate(0,0,.001),
				LinAlg.scale(size), new VzRectangle(new VzMesh.Style(color))));
		
		// The name of the location
		objs.add(new VisChain(LinAlg.rotateZ(Math.PI/2), LinAlg.translate(0,-.8*size,.012),
                LinAlg.scale(0.002),
                new VzText(VzText.ANCHOR.CENTER, String.format("<<black>> [%d] %s", id, name))));
	
		if(sensStates.getState("door") != null && sensStates.getState("door").equals("open")){
			// The smaller inner box is only drawn if it is open
			objs.add(new VisChain(LinAlg.translate(0,0,.014),
					LinAlg.scale(size*.9), new VzRectangle(new VzMesh.Style(Color.DARK_GRAY))));
		}
		
		return new VisChain(objs.toArray());
	}

}
