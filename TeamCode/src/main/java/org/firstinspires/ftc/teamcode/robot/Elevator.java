package org.firstinspires.ftc.teamcode.robot;

import static java.lang.Math.abs;
import static java.lang.Math.signum;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

public class Elevator implements RobotModule {
    private LinearOpMode linearOpMode = null;

    public Elevator(LinearOpMode linearOpMode) {
        this.linearOpMode = linearOpMode;
    }
    private double target = 0;
    private boolean upDown = false;
    private DcMotorEx motorLift = null;
    private Servo servoLift = null;
    public void init(){
        motorLift = linearOpMode.hardwareMap.get(DcMotorEx.class, "E1");
        motorLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }
    public void upElevator(double dist){
        target = dist;
        upDown = true;
    }
    public void downElevator() {
        target = 0;
        upDown = false;
    }
    public boolean queuebool = true;
    public boolean line(){
        return queuebool;
    }
    public void update(){
        if (upDown){
            if(motorLift.getCurrentPosition() < target){
                motorLift.setPower(1);
                queuebool = false;
            }
            else {
                motorLift.setPower(0);
                queuebool = true;
            }
        }else {
            if(motorLift.getCurrentPosition() > target){
                motorLift.setPower(-1);
                queuebool = false;
            }
            else {
                queuebool = true;
                motorLift.setPower(0);
            }
        }
    }
}
