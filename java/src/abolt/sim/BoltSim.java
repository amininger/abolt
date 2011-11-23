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

public class BoltSim extends Simulator
{
    public BoltSim(GetOpt gopt)
    {
        super(gopt);

        {
            VzGrid.addGrid(vw, new VzGrid(new VzLines.Style(Color.gray, 1)));
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
