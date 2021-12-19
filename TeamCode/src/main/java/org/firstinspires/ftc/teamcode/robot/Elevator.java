package org.firstinspires.ftc.teamcode.robot;

import static java.lang.Math.abs;
import static java.lang.Math.signum;


import static org.firstinspires.ftc.teamcode.VariablesDashboard.Elevator.*;
import static org.firstinspires.ftc.teamcode.VariablesDashboard.ManipulatorConfig.AutoTele;
import static org.firstinspires.ftc.teamcode.VariablesDashboard.ManipulatorConfig.positionServoDown;
import static org.firstinspires.ftc.teamcode.VariablesDashboard.ManipulatorConfig.positionServoUp;
import static org.firstinspires.ftc.teamcode.VariablesDashboard.ManipulatorConfig.positonServoForElevator;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.R;

public class Elevator implements RobotModule {

    private Servo servoElevator = null;
    private LinearOpMode linearOpMode = null;

    public Elevator(LinearOpMode linearOpMode) {
        this.linearOpMode = linearOpMode;
    }

    private double target = 0;
    private boolean upDown = false;
    public DcMotorEx motorLift = null;
    private Servo servoLift = null;
    private DistanceSensor distance = null;
    public DigitalChannel limitSwitch = null;

    public void init() {
        limitSwitch = linearOpMode.hardwareMap.get(DigitalChannel.class, "limitSwitch");
        limitSwitch.setMode(DigitalChannel.Mode.INPUT);
        motorLift = linearOpMode.hardwareMap.get(DcMotorEx.class, "E1");
        motorLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        servoElevator = linearOpMode.hardwareMap.get(Servo.class, "UpDown");
        distance = linearOpMode.hardwareMap.get(DistanceSensor.class, "distance");
        motorLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorLift.setDirection(DcMotorEx.Direction.REVERSE);
    }

    public void resetEncoderElevator() {
        motorLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public boolean queuebool = true;

    public boolean line() {
        return queuebool;
    }

    public boolean ejectMinerals = false;
    public double moveServo = positionServoDown;
    public ElapsedTime servoTimer = new ElapsedTime();

    public void MoveServoForElevator(boolean ejectMinerals) { //manipulatorUp //manipulatorDown //positonServoForElevator
        if (this.ejectMinerals != ejectMinerals)
            servoTimer.reset();
        this.ejectMinerals = ejectMinerals;
    }

    public double getElevatorPosition() {
        return motorLift.getTargetPosition();
    }

    public enum ElevatorPosition {
        DOWN, MIDDLE, UP
    }

    public void ElevatorPosition(ElevatorPosition direction) {
        switch (direction) {
            case DOWN:
                target = downTargetElevator;
                break;
            case MIDDLE:
                target = middleTargetElevator;
                break;
            case UP:
                target = upTargetElevator;
                break;
        }
        queuebool = false;  //7
    }

    private double liftEncoderOffset = 0;

    private double getLiftEncoderPosition() {
        return motorLift.getCurrentPosition() - liftEncoderOffset;
    }

    public void update() {
       /* if(AutoTele = true) {
            if (RobotModules.brush.ledMotor.getPower() > 0) ElevatorPosition(ElevatorPosition.UP);
            if (RobotModules.brush.ledMotor.getPower() <= 0)
                ElevatorPosition(ElevatorPosition.DOWN);
        }*/
        servoElevator.setPosition(ejectMinerals ?
                positionServoDown : (target == downTargetElevator ? positionServoUp : positonServoForElevator));
        if (target == downTargetElevator) {
            if (!limitSwitch.getState()){
                motorLift.setPower(-1);
                queuebool = false;
            }
            else {
                motorLift.setPower(0);
                resetEncoderElevator();
                queuebool = true;
            }
        }
        else {
            double error = target - motorLift.getCurrentPosition();
            double kP = 0.2;

            if (abs(error) > 20) {
                motorLift.setPower(error * kP);
                queuebool = false;
            } else {
                motorLift.setPower(0);
                queuebool = (servoTimer.seconds() > bucketServoDelay);
            }
        }
    }
}
