package abolt.arm;

import java.io.*;
import java.util.*;

import lcm.lcm.*;

import april.config.*;
import april.dynamixel.*;
import april.jserial.*;
import april.util.*;
import april.jmat.*;

import abolt.lcmtypes.*;

public class ArmDriver implements LCMSubscriber, Runnable
{
    AbstractServo servos[] = new AbstractServo[6];
    LCM lcm = LCM.getSingleton();

    ExpiringMessageCache<dynamixel_command_list_t> cmdCache;

    public ArmDriver(Config config)
    {
        AbstractBus bus;
        String device;
        String name;
        if (config != null) {
            device = config.getString("arm.device");
            name = config.getString("arm.arm_version");
        } else {
            device = "/dev/ttyUSB0";
            name = null;
        }

        if (device.equals("sim")) {
            /*SimBus sbus = new SimBus(10);
            sbus.addMX64(0);
            sbus.addMX106(1);
            sbus.addMX64(2);
            sbus.addMX22(3);
            sbus.addAX12(4);
            sbus.addAX12(5);
            bus = sbus;*/
            bus = null;
            System.err.println("ERR: Simulation currently not supported");
            System.exit(1);
        } else {
            JSerial js = null;
            try {
                js = new JSerial(device, 1000000);
                js.setCTSRTS(true);
            } catch (IOException ioex) {
                System.err.println("ERR: "+ioex);
                ioex.printStackTrace();
            }
            bus = new SerialBus(js);
        }

        // self-test
        for (int id = 0; id < 6; id++) {
            servos[id] = bus.getServo(id);
            if (servos[id] == null)  {
                System.out.printf("Could not communicate with servo %d\n", id);
                System.exit(-1);
            }
            // handle PID
            if (servos[id] instanceof MXSeriesServo && config != null) {
                int[] pid = config.getInts("arm."+name+".r"+id+".pid", null);
                if (pid != null) {
                    MXSeriesServo mx = (MXSeriesServo)(servos[id]);
                    mx.setPID((byte)pid[0], (byte)pid[1], (byte)pid[2]);
                    System.out.printf("Servo %d : set PID to [%d %d %d]\n", id, pid[0], pid[1], pid[2]);
                }
            }
            System.out.printf("Servo %d : %s present!\n", id, servos[id].getClass().getSimpleName());
        }

        cmdCache = new ExpiringMessageCache<dynamixel_command_list_t>(0.25);

        lcm.subscribe("ARM_COMMAND", this);

        new StatusThread().start();
    }

    public void messageReceived(LCM lcm, String channel, LCMDataInputStream ins)
    {
        try {
            messageReceivedEx(lcm, channel, ins);
        } catch (IOException ex) {
            System.out.println("ex: "+ex);
        }
    }

    public void messageReceivedEx(LCM lcm, String channel, LCMDataInputStream ins) throws IOException
    {
        if (channel.equals("ARM_COMMAND")) {
            dynamixel_command_list_t cmdlist = new dynamixel_command_list_t(ins);
            if (cmdlist.len != 6)
                System.out.println("WRN: Invalid command length received");
            else {
                synchronized(this) {
                    cmdCache.put(cmdlist, TimeUtil.utime());
                    this.notify();
                }
            }
        }
    }

    public void run()
    {
        System.out.println("Starting ArmDriver thread");
        dynamixel_command_list_t cmdlist;
        dynamixel_command_list_t lastCmdList = new dynamixel_command_list_t();
        lastCmdList.len = 6;
        lastCmdList.commands = new dynamixel_command_t[6];
        for (int id = 0; id < 6; id++)
            lastCmdList.commands[id] = null;

        while (true) {
            synchronized(this) {
                cmdlist = cmdCache.get();
                while (cmdlist == null) {
                    try {
                        this.wait();
                    } catch (InterruptedException ex) {
                    }
                    cmdlist = cmdCache.get();
                }
            }
            for (int id = 0; id < cmdlist.len; id++) {
                dynamixel_command_t cmd = cmdlist.commands[id];
                dynamixel_command_t lastCmd = lastCmdList.commands[id];

                boolean update = (lastCmd == null || (cmd.utime - lastCmd.utime) > 1000000 ||
                                  lastCmd.position_radians != cmd.position_radians ||
                                  lastCmd.speed != cmd.speed ||
                                  lastCmd.max_torque != cmd.max_torque);

                if (update) {
                    servos[id].setGoal(cmd.position_radians,
                                       Math.max(0,        Math.min(1,       cmd.speed)),
                                       Math.max(0,        Math.min(1,       cmd.max_torque)));
                    lastCmdList.commands[id] = cmd;
                }
            }
        }
    }

    class StatusThread extends Thread
    {
        public void run()
        {
            System.out.println("Starting ArmDriver StatusThread");
            dynamixel_status_list_t dslist = new dynamixel_status_list_t();
            dslist.len = 6;
            dslist.statuses = new dynamixel_status_t[dslist.len];
            long utime = TimeUtil.utime();
            while (true) {
                for (int id = 0; id < dslist.len; id++) {
                    dynamixel_status_t ds = new dynamixel_status_t();

                    AbstractServo.Status s = servos[id].getStatus();

                    ds.utime = TimeUtil.utime();
                    ds.error_flags = s.errorFlags;
                    ds.position_radians = s.positionRadians;
                    ds.speed = s.speed;
                    ds.load = s.load;
                    ds.voltage = s.voltage;
                    ds.temperature = s.temperature;

                    dslist.statuses[id] = ds;
                }

                lcm.publish("ARM_STATUS", dslist);
                double HZ = 15;
                int maxDelay = (int) (1000 / HZ);
                long now = TimeUtil.utime();
                int delay = Math.min((int)((now - utime) / 1000.0), maxDelay);
                utime = now;
                TimeUtil.sleep(maxDelay - delay);
            }
        }
    }

    public static void main(String args[]) throws IOException
    {
        GetOpt gopt = new GetOpt();
        gopt.addString('c', "config", null, "Config file");
        gopt.addBoolean('h', "help", false, "Show this help");

        if (!gopt.parse(args) || gopt.getBoolean("help")) {
            gopt.doHelp();
            return;
        }
        ConfigFile config = null;
        if (gopt.getString("config") != null) {
            config = new ConfigFile(gopt.getString("config"));
        }

        //new ArmDriver(config).run();
        ArmDriver ad = new ArmDriver(config);
        (new Thread(ad)).start();
    }
}
