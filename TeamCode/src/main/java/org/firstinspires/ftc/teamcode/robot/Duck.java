package org.firstinspires.ftc.teamcode.robot;


import static org.firstinspires.ftc.teamcode.opencv.ArucoDetect.centreOfDuck;
import static org.firstinspires.ftc.teamcode.robot.Duck.DuckConfig.autonomousSpinTime;
import static org.firstinspires.ftc.teamcode.robot.Duck.DuckConfig.directionDuck;
import static org.firstinspires.ftc.teamcode.robot.Duck.DuckConfig.finalMotorSpeed;
import static org.firstinspires.ftc.teamcode.robot.Duck.DuckConfig.initialMotorSpeed;
import static org.firstinspires.ftc.teamcode.robot.Duck.DuckConfig.maxAcceleration;
import static org.firstinspires.ftc.teamcode.robot.Duck.DuckConfig.teleOpSpinTime;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.misc.CommandSender;
import org.firstinspires.ftc.teamcode.misc.AllianceColor;
import org.firstinspires.ftc.teamcode.misc.MotorAccelerationLimiter;
import org.firstinspires.ftc.teamcode.misc.StartingPosition;

public class Duck implements RobotModule {
    private final WoENRobot robot;
    private final ElapsedTime duckTimer = new ElapsedTime();
    private boolean queuebool = true;
    private DcMotorEx duckMotor = null;
    private final CommandSender motorCommandSender = new CommandSender((double value) -> duckMotor.setPower(value));
    private final MotorAccelerationLimiter duckAccelerationLimiter = new MotorAccelerationLimiter(motorCommandSender::send, maxAcceleration);
    private boolean shoudSpin = false;
    private boolean teleOpMode = false;

    public Duck(WoENRobot robot) {
        this.robot = robot;
    }

    public void initialize() {
        duckMotor = robot.getLinearOpMode().hardwareMap.get(DcMotorEx.class, "DuckMotor");
        duckMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        duckMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void setFieldPosition(AllianceColor allianceColor, StartingPosition startingPosition) {
        switch (allianceColor) {
            case RED:
                directionDuck = 1;
                switch (startingPosition) {
                    case LEFT:
                        centreOfDuck = 0;
                        break;
                    case RIGHT:
                        centreOfDuck = -21.5;
                        break;
                }
                break;
            case BLUE:
                directionDuck = -1;
                switch (startingPosition) {
                    case LEFT:
                        centreOfDuck = 0;
                        break;
                    case RIGHT:
                        centreOfDuck = -21.5;
                        break;
                }
                break;

        }
    }

    public void setTeleOpMode(boolean teleOpMode) {
        this.teleOpMode = teleOpMode;
    }

    double accelerationCoefficient = 1;

    public void duckSpin(boolean doSpin) {
        if (shoudSpin != doSpin) duckTimer.reset();
        accelerationCoefficient = (finalMotorSpeed - initialMotorSpeed) / teleOpSpinTime;
        shoudSpin = doSpin;
    }

    public boolean actionIsCompleted() {
        return queuebool;
    }

    public void update() {
        if (shoudSpin && duckTimer.seconds() < (teleOpMode ? teleOpSpinTime : autonomousSpinTime)) {
            duckAccelerationLimiter.setSpeed(directionDuck * Range.clip(duckTimer.seconds() * accelerationCoefficient +
                    initialMotorSpeed, initialMotorSpeed, finalMotorSpeed) * robot.battery.getkVoltage());
            queuebool = false;
        } else {
            shoudSpin = false;
            queuebool = true;
            duckAccelerationLimiter.setSpeed(0);
        }
    }

    @Config
    public static class DuckConfig {
        public static double autonomousSpinTime = 2.0;
        public static double teleOpSpinTime = 1.5;
        public static double finalMotorSpeed = 0.72;
        public static double initialMotorSpeed = 0.4;
        public static double maxAcceleration = 7;
        public static double directionDuck = 1;
    }
}
