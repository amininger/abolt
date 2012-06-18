package abolt.arm;

import java.awt.GridLayout;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.io.IOException;

import javax.swing.*;

import lcm.lcm.LCM;
import lcm.lcm.LCMDataInputStream;
import lcm.lcm.LCMSubscriber;

import abolt.lcmtypes.robot_action_t;
import abolt.lcmtypes.robot_command_t;
import april.util.TimeUtil;
	
public class ArmTester extends JFrame implements LCMSubscriber{

    private JLabel idLabel;
    private JLabel statusLabel;
    private LCM lcm = LCM.getSingleton();

    public ArmTester() {
        super("Arm Tester");
        lcm.subscribe("ROBOT_ACTION", this);

		JPanel panel = new JPanel(new GridLayout(0, 1));
		
		idLabel =  new JLabel("-1");
		panel.add(idLabel);
		
		statusLabel = new JLabel("wait");
		panel.add(statusLabel);
	
        JButton grabButton = new JButton("Grab");
        grabButton.addActionListener(new ActionListener()
        {
            @Override
            public void actionPerformed(ActionEvent e)
            {
            	robot_command_t command = new robot_command_t();
            	command.utime = TimeUtil.utime();
            	
            	String id = JOptionPane.showInputDialog(null,
            			  "Enter the ID to grab",
            			  "Grab Command",
            			  JOptionPane.QUESTION_MESSAGE);
            	if(id == null){
            		return;
            	}
                command.action = String.format("GRAB=%s", id);
                command.dest = new double[6];
                lcm.publish("ROBOT_COMMAND", command);
            }
        });
        panel.add(grabButton);
        
        JButton dropButton = new JButton("Drop");
        dropButton.addActionListener(new ActionListener()
        {
            @Override
            public void actionPerformed(ActionEvent e)
            {
            	robot_command_t command = new robot_command_t();
            	command.utime = TimeUtil.utime();
            	
            	String pos = JOptionPane.showInputDialog(null,
            			  "Enter x y to drop at",
            			  "Drop Command",
            			  JOptionPane.QUESTION_MESSAGE);
            	if(pos == null || !pos.contains(" ")){
            		return;
            	}
            	String[] xy = pos.split(" ");
                command.action = "DROP";
                command.dest = new double[]{Double.parseDouble(xy[0]), Double.parseDouble(xy[1]), 0, 0, 0, 0};
                lcm.publish("ROBOT_COMMAND", command);
            }
        });
        panel.add(dropButton);
        
        JButton pointButton = new JButton("Point");
        pointButton.addActionListener(new ActionListener()
        {
            @Override
            public void actionPerformed(ActionEvent e)
            {
            	robot_command_t command = new robot_command_t();
            	command.utime = TimeUtil.utime();
            	
            	String pos = JOptionPane.showInputDialog(null,
            			  "Enter x y to point to",
            			  "Point Command",
            			  JOptionPane.QUESTION_MESSAGE);
            	if(pos == null || !pos.contains(" ")){
            		return;
            	}
            	String[] xy = pos.split(" ");
                command.action = "POINT";
                command.dest = new double[]{Double.parseDouble(xy[0]), Double.parseDouble(xy[1]), 0, 0, 0, 0};
                lcm.publish("ROBOT_COMMAND", command);
            }
        });
        panel.add(pointButton);
        
        JButton homeButton = new JButton("Home");
        homeButton.addActionListener(new ActionListener()
        {
            @Override
            public void actionPerformed(ActionEvent e)
            {
            	robot_command_t command = new robot_command_t();
            	command.utime = TimeUtil.utime();
                command.action = "RESET";
                command.dest = new double[6];
                lcm.publish("ROBOT_COMMAND", command);
            }
        });
        panel.add(homeButton);
        
        JButton actionButton = new JButton("Action");
        actionButton.addActionListener(new ActionListener()
        {
            @Override
            public void actionPerformed(ActionEvent e)
            {
            	robot_command_t command = new robot_command_t();
            	command.utime = TimeUtil.utime();
            	
            	String action = JOptionPane.showInputDialog(null,
            			  "Enter the Action to perform",
            			  "Action Command",
            			  JOptionPane.QUESTION_MESSAGE);
            	if(action == null){
            		return;
            	}
                command.action = action;
                command.dest = new double[6];
                lcm.publish("ROBOT_COMMAND", command);
            }
        });
        panel.add(actionButton);

        this.add(panel);
        this.setSize(200, 400);
        this.setDefaultCloseOperation(EXIT_ON_CLOSE);
        this.setVisible(true);
    }
    
    public void messageReceived(LCM lcm, String channel, LCMDataInputStream ins)
    {
    	if(channel.equals("ROBOT_ACTION")){
    		try {
    			robot_action_t action = new robot_action_t(ins);
				idLabel.setText("" + action.obj_id);
				statusLabel.setText(action.action);
			} catch (IOException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
    	}
    }
    
    public static void main(String[] args){
    	new ArmTester();
    }
}
