package abolt.arm;

import java.util.*;
import java.io.*;
import java.awt.*;
import javax.swing.*;

import lcm.lcm.*;

import april.vis.*;
import april.util.*;

import abolt.lcmtypes.*;

public class ArmStatusPanel extends JScrollPane implements LCMSubscriber
{
    LCM lcm = LCM.getSingleton();

    JPanel panel = new JPanel();

    public ArmStatusPanel()
    {
        this("ARM_STATUS");
    }

    public ArmStatusPanel(String statusChannel)
    {
        super(JScrollPane.VERTICAL_SCROLLBAR_ALWAYS,
              JScrollPane.HORIZONTAL_SCROLLBAR_NEVER);
        setPreferredSize(new Dimension(200,0)); // XXX
        setViewportView(panel);
        lcm.subscribe(statusChannel, this);
    }

    // === LCM =============

    public void messageReceived(LCM lcm, String channel, LCMDataInputStream ins)
    {
        try {
            messageReceivedEx(lcm, channel, ins);
        } catch (IOException ioex) {
            System.err.println("ERR: ArmStatusPanel LCM channel "+channel);
            ioex.printStackTrace();
        }
    }

    public void messageReceivedEx(LCM lcm, String channel, LCMDataInputStream ins) throws IOException
    {
        if (channel.equals("ARM_STATUS")) {
            // Update the panel rendering
            dynamixel_status_list_t dsl = new dynamixel_status_list_t(ins);
            updateStatuses(dsl);
        }
    }

    // === Panel rendering ==========
    synchronized private void updateStatuses(dynamixel_status_list_t dsl)
    {
        if (dsl == null)
            return;

        // Iterate through statuses and put entries in the layout
        //System.out.println("1");
        synchronized (panel) {
            panel.removeAll();
            panel.setLayout(new GridLayout(dsl.len, 1));
        }

        for (int i = 0; i < dsl.len; i++) {
            dynamixel_status_t s = dsl.statuses[i];

            // XXX Figure out how to max a sexier display
            //GridBagLayout bag = new GridBagLayout();
            //GridBagConstraints bc = new GridBagConstraints();
            JPanel statPanel = new JPanel(new GridLayout(5,1));
            statPanel.setBorder(BorderFactory.createTitledBorder("Servo "+i));

            JLabel temp = new JLabel(String.format("Temp [C]: %.2f",
                                                   s.temperature));

            JLabel volt = new JLabel(String.format("Voltage [V]: %.4f",
                                                   s.voltage));

            JLabel load = new JLabel(String.format("Load [-1,1]: %.4f",
                                                   s.load));

            JLabel speed = new JLabel(String.format("Speed [0,1]: %.4f",
                                                    s.speed));

            JLabel pos = new JLabel(String.format("Pos [-Pi,Pi]: %.4f",
                                                  s.position_radians));

            // Error flag handling
            if ((s.error_flags & dynamixel_status_t.ERROR_VOLTAGE) > 0) {
                volt.setForeground(Color.red);
            }

            if ((s.error_flags & dynamixel_status_t.ERROR_ANGLE_LIMIT) > 0) {
                pos.setForeground(Color.red);
            }

            if ((s.error_flags & dynamixel_status_t.ERROR_OVERHEAT) > 0) {
                temp.setForeground(Color.red);
            }

            if ((s.error_flags & dynamixel_status_t.ERROR_OVERLOAD) > 0) {
                load.setForeground(Color.red);
            }

            statPanel.add(pos);
            statPanel.add(speed);
            statPanel.add(load);
            statPanel.add(volt);
            statPanel.add(temp);

            //System.out.println("2");
            synchronized (panel) {
                panel.add(statPanel);
            }
        }

        //setViewportView(panel);

        //add(panel);
        //System.out.println("3");
        synchronized (panel) {
            panel.revalidate();
            panel.repaint();
        }
    }

    public void paint(Graphics g)
    {
        synchronized (panel) {
            super.paint(g);
        }
    }
}
