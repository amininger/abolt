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

    ServoPanel[] panelList = null;
    class ServoPanel extends JPanel
    {
        JLabel pos;
        JLabel speed;
        JLabel load;
        JLabel volt;
        JLabel temp;

        public ServoPanel(String name)
        {
            super();
            setBorder(BorderFactory.createTitledBorder(name));

            pos = new JLabel("Pos [-Pi,Pi]: ");
            speed = new JLabel("Speed [0,1]: ");
            load = new JLabel("Load [-1,1]: ");
            volt = new JLabel("Volt [V]: ");
            temp = new JLabel("Temp [C]: ");

            setLayout(new GridLayout(5, 1));
            add(pos);
            add(speed);
            add(load);
            add(volt);
            add(temp);
        }

        public void setPos(double in, boolean warn)
        {
            pos.setText(String.format("Pos [-Pi,Pi]: %.4f", in));
            if (warn) {
                pos.setForeground(Color.red);
            } else {
                pos.setForeground(Color.black);
            }
        }

        public void setSpeed(double in, boolean warn)
        {
            speed.setText(String.format("Speed [0,1]: %.4f", in));
            if (warn) {
                speed.setForeground(Color.red);
            } else {
                speed.setForeground(Color.black);
            }
        }

        public void setLoad(double in, boolean warn)
        {
            load.setText(String.format("Load [-1,1]: %.4f", in));
            if (warn) {
                load.setForeground(Color.red);
            } else {
                load.setForeground(Color.black);
            }
        }

        public void setTemp(double in, boolean warn)
        {
            temp.setText(String.format("Temp [F]: %.2f", in));
            if (warn) {
                temp.setForeground(Color.red);
            } else {
                temp.setForeground(Color.black);
            }
        }

        public void setVolt(double in, boolean warn)
        {
            volt.setText(String.format("Volt [V}: %.4f", in));
            if (warn) {
                volt.setForeground(Color.red);
            } else {
                volt.setForeground(Color.black);
            }
        }
    }

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

        if (panelList == null) {
            panel.setLayout(new GridLayout(dsl.len, 1));
            panelList = new ServoPanel[dsl.len];
            for (int i = 0; i < dsl.len; i++) {
                panelList[i] = new ServoPanel("Servo "+i);
                panel.add(panelList[i]);
            }
        }


        for (int i = 0; i < dsl.len; i++) {
            dynamixel_status_t s = dsl.statuses[i];

            boolean vErr = ((s.error_flags & dynamixel_status_t.ERROR_VOLTAGE) > 0);
            boolean angErr = ((s.error_flags & dynamixel_status_t.ERROR_ANGLE_LIMIT) > 0);
            boolean heatErr = ((s.error_flags & dynamixel_status_t.ERROR_OVERHEAT) > 0);
            boolean overErr = ((s.error_flags & dynamixel_status_t.ERROR_OVERLOAD) > 0);

            panelList[i].setPos(s.position_radians, angErr);
            panelList[i].setSpeed(s.speed, false);
            panelList[i].setLoad(s.load, overErr);
            panelList[i].setVolt(s.voltage, vErr);
            panelList[i].setTemp(s.temperature, heatErr);
        }
    }
}
