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

import abolt.util.*;

public class BoltSim extends Simulator
{
    public BoltSim(GetOpt gopt)
    {
        super(gopt);

        {
            VzGrid.addGrid(vw, new VzGrid(new VzLines.Style(Color.gray, 1)));
        }

        RenderThread rt = new RenderThread(vw);
        rt.start();
    }

    class RenderThread extends Thread
    {
        int fps = 30;

        VisWorld vw;

        public RenderThread(VisWorld _vw)
        {
            vw = _vw;
        }

        public void run()
        {
            while (true) {

                for (SimObject o: world.objects) {
                    if (o instanceof SimSensable) {
                        SimSensable s = (SimSensable)o;
                        String name = s.getName();
                        if (name.equals("LIGHT_SWITCH")) {
                            SimActionable a = (SimActionable)o;
                            String toggleState = SimUtil.getTokenValue(a.getState(),
                                                                       "TOGGLE");

                            // Render lights XXX hack to expected sim world
                            VisWorld.Buffer vb = vw.getBuffer("eye-candy");
                            vb.setDrawOrder(-1000);

                            VzSquare sq = new VzSquare(new VzMesh.Style(Color.yellow));
                            if (toggleState.equals("ON")) {
                                vb.addBack(new VisChain(LinAlg.scale(1.3),
                                                        LinAlg.translate(1.20, 0.3),
                                                        sq));
                            }
                            vb.swap();
                        }
                    }
                }


                TimeUtil.sleep(1000/fps);
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
