package org.firstinspires.ftc.teamcode.robot;


import static org.firstinspires.ftc.teamcode.robot.Duck.DuckConfig.autonomousSpinTime;
import static org.firstinspires.ftc.teamcode.robot.Duck.DuckConfig.directionDuck;
import static org.firstinspires.ftc.teamcode.robot.Duck.DuckConfig.motorSpeed;
import static org.firstinspires.ftc.teamcode.robot.Duck.DuckConfig.teleOpSpinTime;
import static org.firstinspires.ftc.teamcode.opencv.ArucoDetect.*;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.misc.PositionOnField;
import org.firstinspires.ftc.teamcode.misc.PositionToSearch;

public class Duck implements RobotModule {
    private DcMotor duckMotor = null;

    private final WoENRobot robot;

    public Duck(WoENRobot robot) {
        this.robot = robot;
    }


    public void initialize() {
        duckMotor = robot.getLinearOpMode().hardwareMap.get(DcMotor.class, "DuckMotor");
        duckMotor.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public void redOrBlue(PositionOnField direction, PositionToSearch position) {
        switch (direction) {
            case RED:
                directionDuck = 1;
                switch (position) {
                    case LEFT:
                        centreOfDuck = -21.5;
                    break;
                    case RIGHT:
                        centreOfDuck = 0;
                    break;
                }
                break;
            case BLUE:
                directionDuck = -1;
                switch (position) {
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

    private boolean shoudSpin = false;
    private boolean teleOpMode = false;

    public void setTeleOpMode(boolean teleOpMode) {
        this.teleOpMode = teleOpMode;
    }

    public void duckSpin(boolean doSpin) {
        if (shoudSpin != doSpin) duckTimer.reset();
        shoudSpin = doSpin;
    }

    public boolean queuebool = true;
    private final ElapsedTime duckTimer = new ElapsedTime();

    public boolean actionIsCompleted() {
        return queuebool;
    }

    public void update() {
        if (shoudSpin && duckTimer.seconds() < (teleOpMode ? teleOpSpinTime : autonomousSpinTime)) {
            duckMotor.setPower(directionDuck * motorSpeed * robot.accumulator.getkVoltage());
            queuebool = false;
        } else {
            queuebool = true;
            duckMotor.setPower(0);
        }
    }

    @Config
    public static class DuckConfig {
        public static double autonomousSpinTime = 5.0;
        public static double teleOpSpinTime = 1.9;
        public static double motorSpeed = 0.4;
        public static double directionDuck = 1;
    }
}
