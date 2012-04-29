package abolt.kinect;

import java.util.*;
import java.io.*;
import javax.swing.*;
import java.awt.*;
import april.jmat.*;

import lcm.lcm.*;
import april.vis.*;
import abolt.lcmtypes.*;

public class MMDemo implements LCMSubscriber, VisObject
{

    static LCM lcm = LCM.getSingleton();

    static double ZOFF = 1.0;

    VisChain vtext = new VisPixCoords(VisPixCoords.ORIGIN.CENTER, new VzText(VzText.ANCHOR.CENTER, "No kinect data"));


    kinect_status_t kst;
    kinect_status_t last_kstat;
    VzPoints vp;

    public MMDemo()
    {

        lcm.subscribe("KINECT_STATUS", this);

    }

    public synchronized void messageReceived(LCM lcm, String channgel, LCMDataInputStream ins)
    {
        try {
            kst = new kinect_status_t(ins);

        } catch (IOException ex){

        }
    }

    public void render(VisCanvas vc, VisLayer layer, VisCanvas.RenderInfo rinfo, GL gl)
    {
        kinect_status_t kstat = kst;

        if (kstat == null) {
            vtext.render(vc,layer,rinfo,gl);
            return;
        } else if (kstat != last_kstat){
            double FOV_Y = Math.toRadians(34.2); // based on angular of 57deg
            double FOV_X = Math.toRadians(45.6);

            ColorMapper cm = ColorMapper.makeJet(0,8.5);

            // Step 1: compute points
            // Step 2: compute colors
            ArrayList<double[]> points = new ArrayList<double[]>();
            VisColorData cd = new VisColorData();
            for (int y = 0; y < kstat.HEIGHT; y++)
                for (int x = 0; x < kstat.WIDTH; x++) {
                    int i = y*kstat.WIDTH+x;
                    int d = ((kstat.depth[2*i + 1]&0xff) << 8) | (kstat.depth[2*i]&0xff);
                    double dm = d / 1000.0;
                    int c = 0xff000000 |
                        ((kstat.rgb[3*i+0]&0xff) << 0) |
                        ((kstat.rgb[3*i+1]&0xff) << 8) |
                        ((kstat.rgb[3*i+2]&0xff) << 16);



                    double phi = FOV_Y * y / kstat.HEIGHT - FOV_Y/2;
                    double theta = FOV_X * x / kstat.WIDTH - FOV_X/2;

                    double xp = dm * Math.cos(phi) * Math.sin(theta);
                    double yp = dm * Math.sin(phi);
                    double zp = dm * Math.cos(phi) * Math.cos(theta);
                    double pt [] = {zp,-xp,-yp + ZOFF};

                    if (zp > .4) {
                        points.add(pt);
                        cd.add(c);
                    }
                }

            vp = new VzPoints(new VisVertexData(points), new VzPoints.Style(cd,2));

            kstat = last_kstat;
        }
        vp.render(vc,layer,rinfo,gl);
    }

    public static void main(String args[])
    {

        VisWorld vw = new VisWorld();
        VisLayer vl = new VisLayer(vw);
        VisCanvas vc = new VisCanvas(vl);

        // ((DefaultCameraManager)vl.cameraManager).interfaceMode = 3.0;
        vl.cameraManager.uiLookAt(new double[]{-.6,0,0.8}, new double[]{0,0,0.7}, new double[]{0,0,1}, true);

        vw.getBuffer("foo").addBack(new MMDemo());
        vw.getBuffer("foo").addBack(new VisChain(LinAlg.translate(0,0,ZOFF),
                                                 new VzBox(.1,.25,.1,new VzMesh.Style(Color.blue))));
        vw.getBuffer("foo").swap();

        VzGrid.addGrid(vw);

        JFrame jf = new JFrame("MMDemo");
        jf.add(vc);

        jf.setSize(1000,600);
        jf.setVisible(true);
        jf.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
    }

}