package abolt.vis;

import java.awt.*;
import javax.swing.*;
import java.util.*;

import april.vis.*;
import april.jmat.*;

public class ValueBar implements VisObject
{
    VisChain chain;

    static final double border = 0.01;
    static final double width = 0.2;
    static final double height = 1.0;

    /** Render the bar from [-.5, .5] */
    public ValueBar(double min, double center, double max, double value, Color c)
    {
        assert (max != min);
        VzRectangle backdrop = new VzRectangle(width, height, new VzMesh.Style(Color.darkGray));
        double colorWidth = width - 2*border;
        double colorHeight;

        if (min > center || max < center)
            center = min;

        value = MathUtil.clamp(value, min, max);

        if (value < center)
            colorHeight = Math.min(center-min, center-value);
        else
            colorHeight = Math.min(max-center, value-center);

        double range = max - min;
        double dy;
        colorHeight /= range;
        colorHeight *= (height-2*border);
        if (value > center)
            dy = colorHeight/2;
        else
            dy = -colorHeight/2;
        double regCenter = (center-min)*height/range - 0.5*height;

        // Colored bar for value
        VzRectangle colorbar = new VzRectangle(colorWidth, colorHeight, new VzMesh.Style(c));

        // Line across "center"
        VzLines line = new VzLines(new VisVertexData(new double[] {-width/2, regCenter, 0.002},
                                                     new double[] {width/2, regCenter, 0.002}),
                                   VzLines.LINES,
                                   new VzLines.Style(Color.white, 2));

        // Prep text XXX White obviously won't work everywhere, but
        // dropshadow looks awful. Text color option?
        Formatter f = new Formatter();
        f.format("<<dropshadow=#00000000,white,monospaced>>%.2f", max);
        VzText maxText = new VzText(VzText.ANCHOR.CENTER, f.toString());
        f = new Formatter();
        f.format("<<dropshadow=#00000000,white,monospaced>>%.2f", min);
        VzText minText = new VzText(VzText.ANCHOR.CENTER, f.toString());
        f = new Formatter();
        f.format("<<dropshadow=#00000000,white,monospaced>>%.2f", value);
        VzText valueText = new VzText(VzText.ANCHOR.LEFT, f.toString());



        chain = new VisChain(backdrop,
                             new VisChain(LinAlg.translate(0, dy+(regCenter), 0.001),
                                          colorbar),
                             new VisChain(LinAlg.translate(0, height/2+.1*height, 0.002),
                                          LinAlg.scale(0.01),
                                          maxText),
                             new VisChain(LinAlg.translate(0, -(height/2+.1*height), 0.002),
                                          LinAlg.scale(0.01),
                                          minText),
                             new VisChain(LinAlg.translate(width/2, (regCenter)+2*dy, 0.002),
                                          LinAlg.scale(0.01),
                                          valueText),
                             line);
    }

    public void render(VisCanvas vc, VisLayer vl, VisCanvas.RenderInfo rinfo, GL gl)
    {
        chain.render(vc, vl, rinfo, gl);
    }

    static public void main(String args[])
    {
        VisWorld vw = new VisWorld();
        VisLayer vl = new VisLayer(vw);
        VisCanvas vc = new VisCanvas(vl);

        vl.cameraManager.fit2D(new double[]{-2,-2}, new double[]{2,2}, true);

        JFrame jf = new JFrame("ValueBar");
        jf.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        jf.setLayout(new BorderLayout());
        jf.setSize(600, 400);
        jf.add(vc);
        jf.setVisible(true);

        ValueBar v1, v2, v3, v4, v5;
        Random r = new Random(52897342);
        v1 = new ValueBar(-3, 0, 3, r.nextGaussian(), Color.red);
        v2 = new ValueBar(-1, .5, 1, r.nextGaussian(), Color.green);
        v3 = new ValueBar(-50, 0, 50, 25*r.nextGaussian(), Color.blue);
        v4 = new ValueBar(-50, 25, 100, 50*r.nextGaussian(), Color.orange);
        v5 = new ValueBar(-100, 0, 250, 100*r.nextGaussian(), Color.magenta);

        VisWorld.Buffer vb = vw.getBuffer("buf");
        vb.addBack(new VisChain(LinAlg.translate(2, 0, 0), v5));
        vb.addBack(new VisChain(LinAlg.translate(1, 0, 0), v1));
        vb.addBack(v2);
        vb.addBack(new VisChain(LinAlg.translate(-1, 0, 0), v3));
        vb.addBack(new VisChain(LinAlg.translate(-2, 0, 0), v4));


        vb.swap();

    }


}
