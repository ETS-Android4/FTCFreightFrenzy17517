package org.firstinspires.ftc.teamcode.robot;

import static org.firstinspires.ftc.teamcode.VariablesDashboard.ManipulatorConfig.manipulatorDown;
import static org.firstinspires.ftc.teamcode.VariablesDashboard.ManipulatorConfig.*;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

public class Manipulator implements RobotModule {

    private Servo servoElevator = null;
    private LinearOpMode linearOpMode = null;

    public Manipulator(LinearOpMode linearOpMode) {
        this.linearOpMode = linearOpMode;
    }

    public boolean queuebool = true;
    public boolean line(){
        return queuebool;
    }
    public void init() {
        servoElevator = linearOpMode.hardwareMap.get(Servo.class, "UpDown");
    }

    public boolean positionUpDown = false;
    public void MoveServo(boolean upDown) {
        positionUpDown = upDown;
    }
    public void MoveServoForElevator(){
        servoElevator.setPosition(positonServoForElevator);
    }
    public void update() {
        if (positionUpDown) {
            queuebool = false;
            servoElevator.setPosition(manipulatorUp);
        }
         else{
            servoElevator.setPosition(manipulatorDown);
            queuebool = true;
         }
        }
    }
