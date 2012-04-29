package abolt.lcm;

import java.awt.*;
import java.awt.image.*;
import java.awt.event.*;
import javax.swing.*;
import java.io.*;

import lcm.lcm.*;
import lcm.spy.*;

import april.util.*;
import april.jmat.*;
import april.vis.*;

import abolt.lcmtypes.*;
import abolt.vis.*;

/** A plugin for visualizing kinect_status_t data */
public class KinectPlugin implements SpyPlugin
{
    public boolean canHandle(long fingerprint)
    {
        return fingerprint == kinect_status_t.LCM_FINGERPRINT;
    }

    public Action getAction(JDesktopPane jdp, ChannelData cd)
    {
        return new KinectAction(jdp, cd);
    }

    class KinectAction extends AbstractAction
    {
        JDesktopPane jdp;
        ChannelData cd;

        public KinectAction(JDesktopPane jdp_, ChannelData cd_)
        {
            cd = cd_;
            jdp = jdp_;
        }

        public void actionPerformed(ActionEvent e)
        {
            Viewer v = new Viewer(cd);
            jdp.add(v);
            v.toFront();
        }
    }

    class Viewer extends JInternalFrame implements LCMSubscriber
    {
        ChannelData cd;
        JImage rgb;
        JImage depth;
        VisWorld vw;

        double[] t_gamma;
        double[] cutoffs = new double[]{1.0, 1.2, 1.4, 1.6, 1.8, 2.0};

        public Viewer(ChannelData cd_)
        {
            super("KinectPlugin: "+cd_.name, true, true);
            cd = cd_;

            // Depth mapping
            t_gamma = new double[2048];

            // From Stephane Magnenat
            double k1 = 1.1863;
            double k2 = 2842.5;
            double k3 = 0.1236;
            for (int i = 0; i < 2048; i++) {
                t_gamma[i] = k3 * Math.tan(i / k2 + k1);
            }

            setLayout(new GridLayout(1,3)); // XXX
            rgb = new JImage(null, true);
            depth = new JImage(null, true);
            add(rgb);
            add(depth);

            vw = new VisWorld();
            VisLayer vl = new VisLayer(vw);
            VisCanvas vc = new VisCanvas(vl);
            vl.cameraManager.fit2D(new double[] {-2,-2},
                                   new double[] {2, 2},
                                   true);
            add(vc);

            setSize(3*kinect_status_t.WIDTH, kinect_status_t.HEIGHT);
            setVisible(true);

            LCM.getSingleton().subscribe(cd.name, this);
        }

        public void messageReceived(LCM lcm, String channel, LCMDataInputStream ins)
        {
            kinect_status_t ks;
            try {
                ks = new kinect_status_t(ins);
            } catch (IOException ex) {
                ex.printStackTrace();
                return;
            }

            // === Make RGB and D images ===
            BufferedImage imRGB = new BufferedImage(ks.WIDTH,
                                                    ks.HEIGHT,
                                                    BufferedImage.TYPE_INT_RGB);
            BufferedImage imD = new BufferedImage(ks.WIDTH,
                                                  ks.HEIGHT,
                                                  BufferedImage.TYPE_INT_RGB);
            int[] bufRGB = ((DataBufferInt)(imRGB.getRaster().getDataBuffer())).getData();
            int[] bufD = ((DataBufferInt)(imD.getRaster().getDataBuffer())).getData();

            for (int i = 0; i < bufRGB.length; i++) {
                bufRGB[i] = 0xff000000 |
                            ((ks.rgb[3*i+0]&0xff) << 16) |
                            ((ks.rgb[3*i+1]&0xff) << 8) |
                            (ks.rgb[3*i+2]&0xff);

                int d = ((ks.depth[2*i+1]&0xff) << 8) |
                        (ks.depth[2*i+0]&0xff);
                double m = d2m(d);

                // XXX Improved color mapping. Optimal range is ~0.8m - 3.5m
                // white -> close
                // red
                // orange
                // yellow
                // green
                // blue
                // magenta
                // black -> bad values
                if (m < 0) {
                    bufD[i] = 0;
                    continue;
                }
                int r, g, b;
                if (m < cutoffs[0]) {
                    r = 0xff;
                    g = 0xff - (int) (0xff * m / cutoffs[0]);
                    b = 0xff - (int) (0xff * m / cutoffs[0]);
                } else if (m < cutoffs[1]) {
                    r = 0xff;
                    g = 0xff - (int) (0xff * ((cutoffs[1] - m) / (cutoffs[1] - cutoffs[0])));
                    b = 0;
                } else if (m < cutoffs[2]) {
                    r = (int) (0xff * ((cutoffs[2] - m) / (cutoffs[2] - cutoffs[1])));
                    g = 0xff;
                    b = 0;
                } else if (m < cutoffs[3]) {
                    r = 0;
                    g = (int) (0xff * ((cutoffs[3] - m) / (cutoffs[3] - cutoffs[2])));
                    b = 0xff - (int) (0xff * ((cutoffs[3] - m) / (cutoffs[3] - cutoffs[2])));
                } else if (m < cutoffs[4]) {
                    r = 0xff - (int) (0xff * ((cutoffs[4] - m) / (cutoffs[4] - cutoffs[3])));
                    g = 0;
                    b = 0xff;
                } else if (m < cutoffs[5]) {
                    r = (int) (0xff * ((cutoffs[5] - m) / (cutoffs[5] - cutoffs[4])));
                    g = 0;
                    b = (int) (0xff * ((cutoffs[5] - m) / (cutoffs[5] - cutoffs[4])));
                } else {
                    r = 0;
                    g = 0;
                    b = 0;
                }

                bufD[i] = 0xff000000 | (r << 16) | (g << 8) | b;
            }

            rgb.setImage(imRGB);
            depth.setImage(imD);
            // =====================================

            // === Create visualizer for accelerometer data ===
            {
                VisWorld.Buffer vb = vw.getBuffer("accerometer");
                ValueBar barX = new ValueBar(-15, 0, 15, ks.dx, Color.red);
                ValueBar barY = new ValueBar(-15, 0, 15, ks.dy, Color.green);
                ValueBar barZ = new ValueBar(-15, 0, 15, ks.dz, Color.blue);

                // Text for each axis XXX

                vb.addBack(new VisChain(LinAlg.translate(-1, 0, 0),
                                        barX));
                vb.addBack(new VisChain(barY));
                vb.addBack(new VisChain(LinAlg.translate(1, 0, 0),
                                        barZ));

                vb.swap();
            }

        }

        // Convert depth value to meters
        double d2m(int d)
        {
            // if (d >= 2047)
            //     return -1;
            return d / 1000.0;//return t_gamma[d];
        }
    }
}
