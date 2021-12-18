package org.firstinspires.ftc.teamcode.robot;

import static java.lang.Math.abs;
import static java.lang.Math.signum;


import static org.firstinspires.ftc.teamcode.VariablesDashboard.Elevator.*;
import static org.firstinspires.ftc.teamcode.VariablesDashboard.ManipulatorConfig.positionServoDown;
import static org.firstinspires.ftc.teamcode.VariablesDashboard.ManipulatorConfig.positionServoUp;
import static org.firstinspires.ftc.teamcode.VariablesDashboard.ManipulatorConfig.positonServoForElevator;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.ElapsedTime;

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
    private ElapsedTime servoTimer = new ElapsedTime();

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
        linearOpMode.telemetry.addData("Rot ebal", liftEncoderOffset);
        servoElevator.setPosition(ejectMinerals ?
                positionServoDown : (target == downTargetElevator ? positionServoUp : positonServoForElevator));
        if (target == downTargetElevator) {
            if (limitSwitch.getState()) {
                liftEncoderOffset = motorLift.getCurrentPosition();
                motorLift.setPower(0.0);
            }
            else{
                motorLift.setPower(-1.0);
            }
        }
        else {
            double error = target - getLiftEncoderPosition();
            double kP = 0.1;
            if (limitSwitch.getState()) {
                motorLift.setPower(0);
                resetEncoderElevator();
            }

            if (abs(error) > 50) {
                motorLift.setPower(error * kP);
                queuebool = false;
            } else {
                motorLift.setPower(0);
                queuebool = (servoTimer.seconds() > bucketServoDelay);
            }
        }
    }
}
