package org.firstinspires.ftc.teamcode.robot;

import static org.firstinspires.ftc.teamcode.VariablesDashboard.ManipulatorConfig.manipulatorDown;
import static org.firstinspires.ftc.teamcode.VariablesDashboard.ManipulatorConfig.manipulatorUp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

public class Manipulator implements RobotModule {

    private Servo liftMotor = null;
    private LinearOpMode linearOpMode = null;

    public Manipulator(LinearOpMode linearOpMode) {
        this.linearOpMode = linearOpMode;
    }

    public boolean queuebool = true;
    public boolean line(){
        return queuebool;
    }
    public void init() {
        liftMotor = linearOpMode.hardwareMap.get(Servo.class, "UpDown");
    }

    public boolean positionUpDown = false;
    public void MoveServo(boolean upDown) {
        positionUpDown = upDown;
    }
    public void update() {
        if (positionUpDown){
            if(liftMotor.getPosition() < manipulatorUp) {
                queuebool = false;
                liftMotor.setPosition(manipulatorUp);
            } else{
                queuebool = true;
            }
        } else {
            liftMotor.setPosition(manipulatorDown);
            queuebool = true;
        }
    }
}
