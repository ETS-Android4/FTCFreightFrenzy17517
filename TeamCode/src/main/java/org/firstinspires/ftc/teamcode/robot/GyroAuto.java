package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.concurrent.TimeUnit;

public class GyroAuto implements RobotModule {
    private boolean gyroTriggered = false;
    private WoENRobot robot = null;
    private final ElapsedTime gyroTimer = new ElapsedTime();
    private double tiltOffset;
    private boolean gyroTiltDetected = false;

    public GyroAuto(WoENRobot robot) {
        this.robot = robot;
    }

    public boolean isGyroTriggered() {
        return gyroTriggered;
    }

    public void resetGyroTrigger() {
        gyroTriggered = false;
    }

    public void initialize() {
        tiltOffset = robot.gyro.getOrientation().thirdAngle;
    }

    public void update() {
        if (Math.abs(getTilt()) > 5.0) {
            gyroTiltDetected = true;
            gyroTimer.reset();
        }
        if (gyroTiltDetected && gyroTimer.time(TimeUnit.SECONDS) > 0.5) {
            gyroTriggered = true;
            gyroTiltDetected = false;
        }
    }

    private double getTilt() {
        return robot.gyro.getOrientation().thirdAngle - tiltOffset;
    }
}
