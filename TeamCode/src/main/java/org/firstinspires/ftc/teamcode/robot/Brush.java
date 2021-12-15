package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.opmodes.TeleOpOneGamepad;


public class Brush implements RobotModule{
    private DcMotorEx brushMotor = null;
    private LinearOpMode linearOpMode = null;

    public Brush(LinearOpMode linearOpMode) {
        this.linearOpMode = linearOpMode;
    }
    public void init() {
        brushMotor = linearOpMode.hardwareMap.get(DcMotorEx.class, "BrushMotor");
        brushMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        brushMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        brushMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        brushMotor.setDirection(DcMotorEx.Direction.REVERSE);
    }
    public boolean intake = false;
    public void brushMotorMove(boolean intake){
        this.intake = intake;
    }
    public void update(){
        if(intake){ brushMotor.setPower(1); }
        else{ brushMotor.setPower(0); }
    }
    public void updateTele(boolean t1){
        if(t1){update();}
    }
    public boolean line(){
        return true;
    }
}
