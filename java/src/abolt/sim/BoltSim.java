package abolt.sim;

import java.io.*;
import java.util.*;
import java.awt.*;
import java.awt.image.*;
import javax.swing.*;

import april.config.*;
import april.util.*;
import april.jmat.*;
import april.vis.*;
import april.sim.*;

public class BoltSim
{
    VisWorld vw = new VisWorld();
    VisLayer vl = new VisLayer(vw);
    VisCanvas vc = new VisCanvas(vl);

    SimWorld world;

    public BoltSim(GetOpt gopt)
    {

        try {
            Config config = new Config();
            if (gopt.wasSpecified("config"))
                config = new ConfigFile(EnvUtil.expandVariables(gopt.getString("config")));

            if (gopt.getString("world").length() > 0) {
                String worldFilePath = EnvUtil.expandVariables(gopt.getString("world"));
                this.world = new SimWorld(worldFilePath, config);
            } else {
                this.world = new SimWorld(config);
            }

        } catch (IOException ex) {
            System.out.println("ex: "+ex);
            ex.printStackTrace();
            return;
        }


        {
            VzGrid.addGrid(vw).groundColor = Color.white;
        }

        JFrame jf = new JFrame("VisTest");
        jf.add(vc);
        jf.setSize(800,800);
        jf.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        jf.setVisible(true);


        vl.backgroundColor = Color.white;
        vl.cameraManager.fit2D(new double[]{1,-.5}, new double[]{2.2,1.2}, true);

        vw.getBuffer("simworld").addFront(new VisSimWorld());


        // XXXXX Kitchen
        {
            VisWorld.Buffer vb = vw.getBuffer("Kitchen");
            vb.addBack(new VzLines(new VisVertexData(SimBoltRobot.kitchenBox),
                                   new VisConstantColor(Color.red),4, VzLines.TYPE.LINE_LOOP));

            vb.addBack(new VzLines(new VisVertexData(SimBoltRobot.stoveBox),
                                   new VisConstantColor(Color.blue),4, VzLines.TYPE.LINE_LOOP));


            vb.addBack(new VisChain(LinAlg.translate((SimBoltRobot.kitchen[0][0] +SimBoltRobot.kitchen[1][0])/2,
                                                     SimBoltRobot.kitchen[1][1],
                                                     0),
                                    LinAlg.scale(.003),
                                    new VzText(VzText.ANCHOR.TOP,
                                               "<<sansserif-12, red>> Kitchen")));

            vb.addBack(new VisChain(LinAlg.translate((SimBoltRobot.stove[0][0] +SimBoltRobot.stove[1][0])/2,
                                                     SimBoltRobot.stove[1][1],
                                                     0),
                                    LinAlg.scale(.003),
                                    new VzText(VzText.ANCHOR.TOP,
                                               "<<sansserif-12, blue>> Stove")));
            vb.swap();
        }


        world.setRunning(true);
    }


    class VisSimWorld implements VisObject
    {
        public void render(VisCanvas vc, VisLayer layer, VisCanvas.RenderInfo rinfo, GL gl)
        {
            synchronized(world) {
                for (SimObject obj : world.objects) {
                    VisChain v = new VisChain(obj.getPose(), obj.getVisObject());
                    v.render(vc, layer, rinfo, gl);
                }
            }


            // Hack
            {
                // VisWorld.Buffer vb = vw.getBuffer("Kitchen-light");
                BufferedImage im = new BufferedImage(1,1,BufferedImage.TYPE_INT_ARGB);
                im.setRGB(0,0, SimBoltRobot.kitchenLight? 0xffffff00: 0x33333333);
                // vb.addBack(

                new VisLighting(false,new VzImage(new VisTexture(im,VisTexture.NO_ALPHA_MASK),
                            SimBoltRobot.kitchenBox,
                            new double[][] { { 0, 0 },
                                             { 1, 0 },
                                             { 1, 1 },
                                             { 0, 1 }}, Color.white)).render(vc,layer,rinfo,gl);

                // vb.swap();

            }
        }
    }


    public static void main(String args[])
    {
        GetOpt gopt = new GetOpt();
        gopt.addBoolean('h', "help", false, "Show this help");
        gopt.addString('w', "world", "", "World file");
        gopt.addString('c', "config", "", "Configuration file");
        gopt.addBoolean('\0', "start", false, "Start simulation automatically");
        gopt.addInt('\0', "fps", 10, "Maximum frame rate");

        if (!gopt.parse(args) || gopt.getBoolean("help") || gopt.getExtraArgs().size() > 0) {
            gopt.doHelp();
            return;
        }

        BoltSim editor = new BoltSim(gopt);
    }


}