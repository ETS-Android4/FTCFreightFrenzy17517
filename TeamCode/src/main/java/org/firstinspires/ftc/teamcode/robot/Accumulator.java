package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.lynx.LynxVoltageSensor;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.misc.TimedSensorQuery;

public class Accumulator implements RobotModule {


    WoENRobot robot;

    private double kVoltage = 1;

    public Accumulator(WoENRobot robot) {
        this.robot = robot;
    }

    VoltageSensor controlHubVoltageSensor = null;


    private final TimedSensorQuery timedVoltageSensor = new TimedSensorQuery(() -> controlHubVoltageSensor.getVoltage(),7);

    public double getkVoltage() {
        return kVoltage;
    }

    public void update() {
        kVoltage = 13 / timedVoltageSensor.getValue();
    }

    public void initialize() {
        LynxModule controlHub = robot.getLinearOpMode().hardwareMap.get(LynxModule.class, "Control Hub");
        try {
            controlHubVoltageSensor = new LynxVoltageSensor(robot.getLinearOpMode().hardwareMap.appContext, controlHub);
        } catch (Exception ignored) {
        }
    }

    public boolean actionIsCompleted() {
        return true;
    }

}
