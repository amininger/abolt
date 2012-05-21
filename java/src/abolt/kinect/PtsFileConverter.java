package abolt.kinect;

import java.awt.Color;
import java.awt.image.BufferedImage;
import java.io.*;
import java.util.*;

import abolt.classify.ColorFeatureExtractor;
import abolt.classify.FEUtil;
import abolt.classify.Features;
import abolt.classify.SoarSymbols;
import abolt.classify.Features.FeatureCategory;
import abolt.classify.ShapeFeatureExtractor;
import abolt.classify.SizeFeatureExtractor;

import april.util.*;

/** Take in a pts file and output a feature vector file */
public class PtsFileConverter
{
    public PtsFileConverter(GetOpt opts)
    {
        FileInputStream fin;
        DataInputStream ins = null;
        FileWriter fout = null;
        try {
        	
        	String inFileString = opts.getString("infile");
        	String[] inFiles = inFileString.split(",");
            fout = new FileWriter(opts.getString("outfile"), false);
        	for(int i = 0; i < inFiles.length; i++){
                fin = new FileInputStream(inFiles[i]);
                ins = new DataInputStream(fin);

                FeatureCategory type;
                if(opts.getString("type").equals("color")){
                	type = FeatureCategory.COLOR;
                } else if(opts.getString("type").equals("shape")){
                	type = FeatureCategory.SHAPE;
                } else {
                	type = FeatureCategory.SIZE;
                }
                
                if (ins != null && fout != null) {
                    convertFile(ins, fout, type);
                }
                ins.close();
                fin.close();
        	}

        } catch (Exception ex) {
            System.err.println("ERR: "+ex);
            ex.printStackTrace();
        }

    }

    private void convertFile(DataInputStream ins, FileWriter fout, FeatureCategory type)
    {
        BinaryStructureReader bsr = new BinaryStructureReader(ins);
        PrintWriter pwout;
        try {
            pwout = new PrintWriter(fout);
        } catch (Exception ioex) {
            ioex.printStackTrace();
            return;
        }
        

        try {
            while (true) {
                // Read in data
                int nlabels = bsr.readInt();
                bsr.blockBegin();
                ArrayList<String> labels = new ArrayList<String>();
                for (int i = 0; i < nlabels; i++) {
                    String label = bsr.readString();
                    labels.add(label);
                }
                bsr.blockEnd();

                int npoints = bsr.readInt();
                ArrayList<double[]> points = new ArrayList<double[]>();
                bsr.blockBegin();
                for (int i = 0; i < npoints; i++) {
                    points.add(bsr.readDoubles());
                }
                bsr.blockEnd();
                
                ArrayList<Double> features = Features.getFeatures(type, points);
                String featureString = FEUtil.featuresToString(features);
                
                boolean hasLabel = false;
                for(int i = 0; i < labels.size(); i++){
                	String featureSymbol = SoarSymbols.getSymbol(type, labels.get(i).toLowerCase());
                	if(featureSymbol != null){
                		featureString += String.format(" {%s}\n", featureSymbol);
                		hasLabel = true;
                		break;
                	}
                }
                if(hasLabel){
                	pwout.print(featureString);
                	pwout.flush();
                }
            }
        } catch (Exception ex) {
        }
    }

    static public void main(String[] args)
    {
        GetOpt opts = new GetOpt();

        opts.addBoolean('h', "help", false, "Show this help screen");
        opts.addString('i', "infile", null, "Input .pts file");
        opts.addString('o', "outfile", null, "Output feature vector file");
        opts.addString('t', "type", "shape", "Type of features to extract: {color, shape}");

        if (!opts.parse(args)) {
            System.err.println("ERR: "+opts.getReason());
            System.exit(1);
        }
        if (opts.getBoolean("help")) {
            opts.doHelp();
            System.exit(1);
        }

        PtsFileConverter pfc = new PtsFileConverter(opts);
    }
}