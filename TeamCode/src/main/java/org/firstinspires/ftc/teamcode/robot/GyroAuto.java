package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

import java.util.Locale;
import java.util.concurrent.TimeUnit;

public class GyroAuto implements RobotModule {
    private WoENRobot robot = null;

    public GyroAuto(WoENRobot robot) {
        this.robot = robot;
    }

    private ElapsedTime gyro_timer = new ElapsedTime();
    private double gyro_in;
    private boolean private_status = false;
    public boolean gyro_status = false;

    public void initialize() {
        gyro_in = robot.gyro.getOrientation().thirdAngle;
    }

    public void update() {
        if (Math.abs(angle()) > 5.0) {
            private_status = true;
            gyro_timer.reset();
        }
        if (private_status && gyro_timer.time(TimeUnit.SECONDS) > 0.5) {
            gyro_status = true;
            private_status = false;
        }
    }

    private double angle() {
        return robot.gyro.getOrientation().thirdAngle - gyro_in;
    }
}
