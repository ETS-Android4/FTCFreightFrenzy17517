package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

import java.util.Locale;
import java.util.concurrent.TimeUnit;

public class Gyro_auto{
    private WoENRobot robot = null;

    public Gyro_auto(WoENRobot robot) { this.robot = robot; }
    private ElapsedTime gyro_timer = new ElapsedTime();
    private double gyro_in;
    private BNO055IMU gyro = null;
    private LedStrip led = new LedStrip(robot);
    private boolean private_status = false;
    public boolean gyro_status  = false;

    public void initialize(){
        gyro_timer.reset();
        gyro_in = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).secondAngle;
    }

    public void reaction(){
        if(robot.bucket.isFreightDetected()){
            if(Math.abs(angle()) > 7.0){ private_status = true; gyro_timer.reset();}
            if(private_status && gyro_timer.time(TimeUnit.SECONDS) > 1.5){
                gyro_status = true;
                private_status = false;
            }
        }
    }

    private double angle(){ return gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).secondAngle - gyro_in; }
}
