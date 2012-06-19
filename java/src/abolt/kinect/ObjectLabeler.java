package abolt.kinect;

import april.vis.*;
import april.jmat.*;
import april.util.*;

import lcm.logging.*;
import lcm.lcm.*;
import abolt.classify.Features;
import abolt.classify.SizeFeatureExtractor;
import abolt.lcmtypes.*;

import java.io.*;
import java.nio.*;
import javax.swing.*;
import java.awt.*;
import java.util.*;
import java.awt.image.*;


class ObjectLabeler // implements LCMSubscriber
{
    static int initialColorThresh = 50;
    static double initialUnionThresh = 0.5;
    static double  initialRansacThresh = .02;
    static double  initialRansacPercent = .1;
    final static int FRAME_WIDTH = 640;
    final static int FRAME_HEIGHT = 480;
    final int MIN_POINTS = 250;

    VisWorld vw, vw2;
    VisLayer vl, vl2;
    VisWorld.Buffer vb;
    JList jlist;
    String[] allAttributes;
    Log log;
    int[] objRefs;
    int currentObject;
    static Segment segment;

    // The most recently accessed kinect status
    kinect_status_t ks;
    // Mapping of attributes to lists of labels
    HashMap<String, ArrayList<String> > labelMap = new HashMap<String, ArrayList<String> >();

    public ObjectLabeler(GetOpt opt)
    {
        try {
            allAttributes = loadLabelConfig(opt.getString("config"));
            assert (allAttributes != null);
        } catch (Exception ex) {
            System.err.println("ERR: "+ex);
            ex.printStackTrace();
        }

        try{
            log = new Log(opt.getString("log"), "r");
        }catch(Exception e){
            System.err.println("Error: "+e);
        }
        final String labelFile = opt.getString("labelFile");

        // Initialize Vis stuff.
        vw = new VisWorld();
        vw2 = new VisWorld();
        vl = new VisLayer(vw);
        vl2 = new VisLayer(vw2);
        double ar = (double)kinect_status_t.WIDTH/(double)kinect_status_t.HEIGHT;
        double pct = 0.25;
        vl2.layerManager = new DefaultLayerManager(vl2, new double[]{0.0, 1.0-pct, pct, pct});
        vl2.cameraManager.fit2D(new double[2], new double[] {kinect_status_t.WIDTH, kinect_status_t.HEIGHT}, true);
        VisCanvas vc = new VisCanvas(vl);
        vc.addLayer(vl2);
        vb = vw.getBuffer("object labeling");
        //Set up initial camera view
        vl.cameraManager.uiLookAt(new double[] {0.0, 0.0, -2.0},// Camera position
                                  new double[] {0.0, 0.0, 0.0},// Point looking at
                                  new double[] {0.0, -1.0, 0.0},// Up
                                  false);

        // Set up labels to choose from and buttons for skipping/commiting
        jlist = new JList(allAttributes);
        JScrollPane scrollPane = new JScrollPane(jlist);
        ParameterGUI pg = new ParameterGUI();
        pg.addButtons("skipF", "Skip Frame", "skipO", "Skip Object", "commit", "Commit Labels");

        pg.addListener(new ParameterListener() {
                public void parameterChanged(ParameterGUI pg, String name) {
                    if (name.equals("skipF")) {
                        getNewFrame();
                    }
                    else if (name.equals("skipO")) {
                        boolean objectExists = selectNewObject();
                        if (!objectExists) getNewFrame();
                    }
                    else if (name.equals("commit")) {
                        commitLabels(labelFile);
                        boolean objectExists = selectNewObject();
                        if (!objectExists) getNewFrame();
                    }
                }
            });


        // JFrame set up
        JFrame frame = new JFrame("Learn Object Labels");
        frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        frame.setLayout(new BorderLayout());
        frame.add(vc, BorderLayout.CENTER);
        frame.add(scrollPane, BorderLayout.EAST);
        frame.add(pg, BorderLayout.SOUTH);
        frame.setSize((int)1.5*FRAME_WIDTH, FRAME_HEIGHT);
        frame.setVisible(true);
    }


    /** Read in a config file detailing the object types expected in this image.
     * If no labels are given, operate off of default label set
     */
    public String[] loadLabelConfig(String filename)
    {
        if (filename != null) {
            try {
                Scanner sc = new Scanner(new File(filename));
                while (sc.hasNextLine()) {
                    String line = sc.nextLine();
                    Scanner lsc = new Scanner(line);

                    String name = lsc.next();
                    ArrayList<String> labels = new ArrayList<String>();
                    while (lsc.hasNext()) {
                        labels.add(lsc.next());
                    }
                    labelMap.put(name, labels);
                }
            } catch (Exception ex) {
                System.out.println("ERR: "+ex);
                ex.printStackTrace();
            }
            String[] temp = new String[1];
            return labelMap.keySet().toArray(temp);
        } else {
            // Use defaults
            String[] attrs = new String[]{"red", "orange", "yellow", "green", "blue", "purple", "square",
                                          "rectangle", "triangle", "arch", "half-cylinder", "cylinder",
                                          "thin", "thick", "small", "medium", "large", "T-shaped",
                                          "L-shaped", "doughnut", "pointy", "rounded", "textured",
                                          "smooth", "cube", "sphere"};
            for (String s: attrs) {
                ArrayList<String> labels = new ArrayList<String>();
                labels.add(s);
                labelMap.put(s, labels);
            }
            return attrs;
        }
    }


    /** Update the parameters by resegmenting the image.  Will use the values of
     ** for thresholds found in the current data aggragator (so updates should be
     ** set first.
     **/
    public void updateParams()
    {
        // Segment the image and return list of segmented objects
        segment.segmentFrame(segment.points, FRAME_WIDTH, FRAME_HEIGHT);
        objRefs = new int[segment.objects.size()];
        Set<Integer> keys = segment.objects.keySet();
        Integer[] keysArray = keys.toArray(new Integer[0]);
        for(int i=0; i<keysArray.length; i++){
            objRefs[i] = keysArray[i];
        }

        // Choose the first object
        selectNewObject();
    }


    /** Load in the next frame from the log. Clears the list of attributes that
     ** were selected, segments the frame, and highlights the first object.**/
    public void getNewFrame()
    {
        jlist.clearSelection();
        currentObject = -1;

        // Read the next kinect message from our cache and transform data into 3D
        try{
            for(int i=0; i<20; i++){
                ks = new kinect_status_t(log.readNext().data);
            }
            segment.points = new ArrayList<double[]>();
            segment.coloredPoints = new ArrayList<double[]>();
            for(int y=0; y<ks.HEIGHT; y++){
                for(int x=0; x<ks.WIDTH; x++){

                    int i = y*ks.WIDTH + x;
                    int d = ((ks.depth[2*i+1]&0xff) << 8) |
                        (ks.depth[2*i+0]&0xff);

                    double[] p = KUtils.getXYZRGB(x, y, d, ks);
                    segment.points.add(p);
                }
            }

            // Segment the image and return list of segmented objects
            segment.segmentFrame(segment.points, FRAME_WIDTH, FRAME_HEIGHT);
            objRefs = new int[segment.objects.size()];
            Set<Integer> keys = segment.objects.keySet();
            Integer[] keysArray = keys.toArray(new Integer[0]);
            for(int i=0; i<keysArray.length; i++){
                objRefs[i] = keysArray[i];
            }

            // Choose the first object
            selectNewObject();
        }
        catch(Exception e){
            System.err.println(e.getMessage());
        }
    }

    /** Choose the next object in the image to draw a box around and label.**/
    public boolean selectNewObject()
    {
        if (currentObject >= objRefs.length-1) return false;

        // Need to highlight the correct attributes in the jlist
        currentObject ++;
        ObjectInfo oi = segment.objects.get(objRefs[currentObject]);
        draw2DImage(oi);
        return true;
    }


    /** Once the user is done selecting labels to apply, tag the feature vector
     ** with what it is a positive example of then write it to a file.
     ** @param labelsFile is the file we're saving examples to.
     **/
    public void commitLabels(String labelsFile)
    {
        // Get the list of labels
        Object[] chosen = jlist.getSelectedValues();
        ArrayList<String> labels = new ArrayList<String>();
        for(int i=0; i < chosen.length; i++){
            labels.addAll(labelMap.get((String)chosen[i]));
        }

        // Get the feature vector
        int id = objRefs[currentObject];
        ArrayList<double[]> points = segment.objects.get(id).points;


        // Write labels and pointclouds to file
        try{
            boolean append = true;
            // Write out po9int clouds for each object
            FileOutputStream fos = new FileOutputStream(labelsFile+".pts", append);
            DataOutputStream dos = new DataOutputStream(fos);
            BinaryStructureWriter w = new BinaryStructureWriter(dos);
            // Write number of features, then all the features in their own block
            w.writeInt(labels.size());
            w.blockBegin();
            for(String s: labels){
                w.writeString(s);
            }
            w.blockEnd();
            // Write number of points, then all the points in their own block
            w.writeInt(points.size());
            w.blockBegin();
            for(double[] p: points){
                w.writeDoubles(p);
            }
            w.blockEnd();
            w.close();
        }
        catch(Exception e){
            System.err.println("ERROR: "+e.getMessage());
        }

        jlist.clearSelection();
    }


    /** Check whether the given label is in list of chosen labels, called on
     ** a String.
     ** @param label we're checking for inclusion.
     ** @param chosen is the list of chosen labels.
     ** @return whether this String describes object.**/
    private boolean labelSelected(String label, String[] chosen)
    {
        boolean selected = false;

        for(int i=0; i<chosen.length; i++){
            if(chosen[i].equals(label)){
                selected = true;
                break;
            }
        }
        return selected;
    }


    /** Drawing a box around the object of interest.**/
    public void draw2DImage(ObjectInfo oi)
    {
        double[] center = oi.getCenter();
        int[] color = oi.avgColor();
        /*System.out.println("RepID: "+oi.repID+
                           "\tCenter: ("+
                           center[0]+","+center[1]+","+center[2]
                           +")\tColor: ("+
                           color[0]+","+color[1]+","+color[2]
                           +")");*/
        if(segment.coloredPoints.size() <= 0){ return; }

        VisColorData cd = new VisColorData();
        VisVertexData vd = new VisVertexData();

        // Add color 2D points
        for(double[] p: segment.coloredPoints){
            vd.add(new double[]{p[0], p[1], 0});
            cd.add((int) p[3]);
        }
        vb.addBack(new VisLighting(false, new VzPoints
                                   (vd, new VzPoints.Style(cd, 1.0))));
        if(oi != null){
            VisVertexData square = new VisVertexData();
            double[] bounds = SizeFeatureExtractor.boundingBox(oi.points);
            square.add(new double[]{bounds[0], bounds[1],0});
            square.add(new double[]{bounds[0], bounds[4],0});
            square.add(new double[]{bounds[3], bounds[4],0});
            square.add(new double[]{bounds[3], bounds[1],0});
            vb.addBack(new VzLines(square, VzLines.LINE_LOOP,
                                  new VzLines.Style(Color.RED, 3)));
        }

        BufferedImage im = new BufferedImage(ks.WIDTH, ks.HEIGHT, BufferedImage.TYPE_3BYTE_BGR);
        byte[] buf = ((DataBufferByte)(im.getRaster().getDataBuffer())).getData();
        for (int i = 0; i < buf.length; i+=3) {
            buf[i] = ks.rgb[i+2];	// B
            buf[i+1] = ks.rgb[i+1];	// G
            buf[i+2] = ks.rgb[i];	// R
        }

        VisWorld.Buffer vbim = vw2.getBuffer("pictureInPicture");
        vbim.addBack(new VzImage(im, VzImage.FLIP));

        vb.swap();
        vbim.swap();
    }


    public static void main(String[] args)
    {
        // Define possible commandline arguments
        GetOpt opts = new GetOpt();
        opts.addBoolean('h', "help", false, "Show this help screen");
        opts.addString('c', "config", null, "Load label config file");
        opts.addString('f', "labelFile", "stdout", "File that holds the labels to automatically add");
        opts.addString('l', "log", "stdout", "Log file");

        if (!opts.parse(args)) {
            System.err.println("ERR: Opts error - " + opts.getReason());
            System.exit(1);
        }
        if (opts.getBoolean("help")) {
            opts.doHelp();
            System.exit(1);
        }

        segment = Segment.getSingleton();
        ObjectLabeler ol = new ObjectLabeler(opts);
    }
}
