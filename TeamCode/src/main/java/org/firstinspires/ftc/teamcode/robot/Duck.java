package org.firstinspires.ftc.teamcode.robot;


import static org.firstinspires.ftc.teamcode.robot.Duck.DuckConfig.directionDuck;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.misc.PositionOnField;

public class Duck implements RobotModule {
    private DcMotor duckMotor = null;

    private WoENRobot robot = null;

    public Duck(WoENRobot robot) {
        this.robot = robot;
    }

    private GyroAuto gyroAuto = new GyroAuto(robot);

    public void initialize() {
        duckMotor = robot.getLinearOpMode().hardwareMap.get(DcMotor.class, "DuckMotor");
        duckMotor.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public double direction = 1;

    public void redOrBlue(PositionOnField direction) {
        switch (direction) {
            case RED:
                directionDuck = 1;
                break;
            case BLUE:
                directionDuck = -1;
                break;
        }

    }

    private boolean doSpin = false;
    public double time = 5;

    public void Teleop() {
        time = 1.9;
    }

    public void duckSpin(boolean ds) {
        if (doSpin != ds) duckTimer.reset();
        doSpin = ds;
    }

    public void setDirection(int direction) {
        this.direction = direction;
    }

    public boolean queuebool = true;
    private ElapsedTime duckTimer = new ElapsedTime();

    public boolean actionIsCompleted() {
        return queuebool;
    }

    public void update() {
        if (doSpin && duckTimer.seconds() < time) {
            duckMotor.setPower(Range.clip(directionDuck * 0.555, -1, 1));
            queuebool = false;
        } else {
            queuebool = true;
            duckMotor.setPower(0);
        }
    }

    @Config
    public static class DuckConfig {
        public static double directionDuck = 1;
    }
}
