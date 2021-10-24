package org.firstinspires.ftc.teamcode.robot;

import static org.firstinspires.ftc.teamcode.VariablesDashboard.ManipulatorConfig.manipulatorCentral;
import static org.firstinspires.ftc.teamcode.VariablesDashboard.ManipulatorConfig.manipulatorDown;
import static org.firstinspires.ftc.teamcode.VariablesDashboard.ManipulatorConfig.manipulatorUp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

public class Manipulator {

        /*
        DigitalChannel endSwitchUp;
        DigitalChannel endSwitchDown;*/
private Servo liftMotor = null;

public void setPosition(int i) {
        }

enum ManipulatorPosition {
    UP, DOWN, CENTRAL
}

    private LinearOpMode linearOpMode = null;
    public Manipulator(LinearOpMode linearOpMode){
        this.linearOpMode = linearOpMode;
    }

    public void init(){
        liftMotor = linearOpMode.hardwareMap.get(Servo.class, "UpDown");
    }

    public void MoveServo(ManipulatorPosition manipulatorPosition) {
        switch (manipulatorPosition){
            case UP: liftMotor.setPosition(manipulatorUp);
            case DOWN: liftMotor.setPosition(manipulatorDown);
            case CENTRAL: liftMotor.setPosition(manipulatorCentral);
        }
    }
}