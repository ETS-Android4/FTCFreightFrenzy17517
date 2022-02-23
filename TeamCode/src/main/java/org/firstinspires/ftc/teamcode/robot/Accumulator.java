package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.lynx.LynxVoltageSensor;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.internal.tfod.RollingAverage;
import org.firstinspires.ftc.teamcode.misc.TimedSensorQuery;

public class Accumulator implements RobotModule {

    WoENRobot robot;

    private double kVoltage = 1;
    private VoltageSensor controlHubVoltageSensor = null;
    private double batteryVoltage = 13.0;
    private final RollingAverage averageBatteryVoltage = new RollingAverage(8, batteryVoltage);
    private final TimedSensorQuery<Double> timedVoltageSensor = new TimedSensorQuery<>(() -> {
        averageBatteryVoltage.add(controlHubVoltageSensor.getVoltage());
        return averageBatteryVoltage.get();
    }, 8);


    public Accumulator(WoENRobot robot) {
        this.robot = robot;
    }

    public double getBatteryVoltage() {
        return batteryVoltage;
    }

    public double getkVoltage() {
        return kVoltage;
    }

    public void update() {
        batteryVoltage = timedVoltageSensor.getValue();
        kVoltage = 13 / batteryVoltage;
    }

    public void initialize() {
        LynxModule controlHub = robot.getLinearOpMode().hardwareMap.get(LynxModule.class, "Control Hub");
        try {
            controlHubVoltageSensor = new LynxVoltageSensor(robot.getLinearOpMode().hardwareMap.appContext, controlHub);
        } catch (Exception ignored) {
        }
    }
}
