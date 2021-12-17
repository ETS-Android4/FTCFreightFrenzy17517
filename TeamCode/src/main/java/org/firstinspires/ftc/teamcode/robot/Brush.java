package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.opmodes.TeleOpOneGamepad;


public class Brush implements RobotModule{
    private DcMotorEx brushMotor = null;
    private LinearOpMode linearOpMode = null;
    private DistanceSensor distance = null;
    public DcMotorEx ledMotor = null;

    public Brush(LinearOpMode linearOpMode) {
        this.linearOpMode = linearOpMode;
    }
    public void init() {
        brushMotor = linearOpMode.hardwareMap.get(DcMotorEx.class, "BrushMotor");
        brushMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        brushMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        brushMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        brushMotor.setDirection(DcMotorEx.Direction.REVERSE);
        distance = linearOpMode.hardwareMap.get(DistanceSensor.class, "distance");
        ledMotor = linearOpMode.hardwareMap.get(DcMotorEx.class,"Led");

    }
    public boolean intake = false;
    public void brushMotorMove(boolean intake){
        this.intake = intake;
    }
    public void update(){
        if(intake && distance.getDistance(DistanceUnit.CM) > 8){
            brushMotor.setPower(1);
            ledMotor.setPower(0);
        }
        else if (intake && distance.getDistance(DistanceUnit.CM ) < 8){
            brushMotor.setPower(-1);
            ledMotor.setPower(1);
        } else {
            brushMotor.setPower(0);
            ledMotor.setPower(0);
        }
    }
    public boolean line(){
        return true;
    }
}
