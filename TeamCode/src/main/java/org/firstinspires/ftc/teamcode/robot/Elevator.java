package org.firstinspires.ftc.teamcode.robot;

import static java.lang.Math.abs;
import static java.lang.Math.signum;


import static org.firstinspires.ftc.teamcode.VariablesDashboard.Elevator.*;
import static org.firstinspires.ftc.teamcode.VariablesDashboard.ManipulatorConfig.*;
import static org.firstinspires.ftc.teamcode.VariablesDashboard.ManipulatorConfig.positionServoDown;
import static org.firstinspires.ftc.teamcode.VariablesDashboard.ManipulatorConfig.positionServoUp;
import static org.firstinspires.ftc.teamcode.VariablesDashboard.ManipulatorConfig.positonServoForElevator;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

public class Elevator implements RobotModule {

    private Servo servoElevator = null;
    private LinearOpMode linearOpMode = null;

    public Elevator(LinearOpMode linearOpMode) {
        this.linearOpMode = linearOpMode;
    }
    private double target = 0;
    private boolean upDown = false;
    private DcMotorEx motorLift = null;
    private Servo servoLift = null;
    public void init() {
        motorLift = linearOpMode.hardwareMap.get(DcMotorEx.class, "E1");
        motorLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        servoElevator = linearOpMode.hardwareMap.get(Servo.class, "UpDown");
    }
    public boolean queuebool = true;
    public boolean line(){
        return queuebool;
    }
    public boolean positionUpDown = false;
    public double moveServo = positionServoDown;
    public void MoveServoForElevator(){ //manipulatorUp //manipulatorDown //positonServoForElevator
        positionUpDown=true;
    }

    public enum ElevatorPosition{
        DOWN, MIDDLE, UP
    }
    public void ElevatorPosition(ElevatorPosition direction) {
        switch (direction) {
            case DOWN:
                target = DownTargetElevator;
            break;
            case MIDDLE:
                target = MiddleTargetElevator;
            break;
            case UP:
                target = UpTargetElevator;
            break;
        }
        upDown = true;
    }
    public void update(){

        servoElevator.setPosition(positionUpDown?
                (target == DownTargetElevator?positionServoUp:positonServoForElevator):positionServoDown);
        double error = target - motorLift.getCurrentPosition();
        double KP = 0.01;
        if(upDown){
            motorLift.setPower(error * KP);
            queuebool = false;

        } else {
            queuebool = true;
        }
    }
}
