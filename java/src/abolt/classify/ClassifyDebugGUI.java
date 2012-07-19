package abolt.classify;

import java.awt.*;
import java.awt.event.*;
import java.util.*;
import javax.swing.*;

import april.util.*;

import abolt.objects.*;
import abolt.classify.Features.FeatureCategory;

/** An interface for rapidly training classifier
 *  for testing.
 **/
public class ClassifyDebugGUI
{
    static JTextField colorField = new JTextField();
    static JButton colorButton = new JButton("Color");
    static JTextField shapeField = new JTextField();
    static JButton shapeButton = new JButton("Shape");
    static JTextField sizeField = new JTextField();
    static JButton sizeButton = new JButton("Size");

    static JTextField idField = new JTextField();
    static JLabel idLabel = new JLabel("Input ID:");
    static JComboBox idComboBox = new JComboBox();

    static Object comboBoxLock = new Object();

    public ClassifyDebugGUI()
    {
        JFrame jf = new JFrame("Classication Debugging Console");
        //jf.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        jf.setLayout(new GridLayout(4,1));

        // ID input
        JPanel idPanel = new JPanel(new GridLayout(1, 2));
        idPanel.add(idLabel);
        idPanel.add(idField);

        // Set up color, shape, and size inputs
        JPanel colorPanel = new JPanel(new GridLayout(1, 2));
        JPanel shapePanel = new JPanel(new GridLayout(1, 2));
        JPanel sizePanel = new JPanel(new GridLayout(1, 2));

        MyListener l = new MyListener();
        colorButton.setActionCommand("color");
        colorButton.addActionListener(l);
        colorPanel.add(colorField);
        colorPanel.add(colorButton);

        shapeButton.setActionCommand("shape");
        shapeButton.addActionListener(l);
        shapePanel.add(shapeField);
        shapePanel.add(shapeButton);

        sizeButton.setActionCommand("size");
        sizeButton.addActionListener(l);
        sizePanel.add(sizeField);
        sizePanel.add(sizeButton);

        jf.add(idPanel);
        jf.add(colorPanel);
        jf.add(shapePanel);
        jf.add(sizePanel);

        jf.pack();
        jf.setVisible(true);
    }

    static class MyListener implements ActionListener
    {
        public void actionPerformed(ActionEvent e)
        {
            FeatureCategory cat;
            String label;
            if (e.getActionCommand().equals("color")) {
                cat = FeatureCategory.COLOR;
                label = colorField.getText();
            } else if (e.getActionCommand().equals("shape")) {
                cat = FeatureCategory.SHAPE;
                label = shapeField.getText();
            } else if (e.getActionCommand().equals("size")) {
                cat = FeatureCategory.SIZE;
                label = sizeField.getText();
            } else {
                System.err.println("ERR: Unknown classifier type");
                return;
            }

            // Find the object
            int repID;
            BoltObject obj;
            synchronized (BoltObjectManager.getSingleton().objects) {
                repID = Integer.valueOf(idField.getText());
                obj = BoltObjectManager.getSingleton().objects.get(repID);
            }

            if (obj == null) {
                System.err.println("ERR: No object by ID "+repID);
                return;
            }

            ClassifierManager.getSingleton().addDataPoint(cat,
                                                          obj.getFeatures(cat),
                                                          label);
        }
    }
}
