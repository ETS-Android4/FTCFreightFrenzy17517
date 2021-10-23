package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.VariablesDashboard.ManipulatorConfig.*;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

public class Manipulator {
    DigitalChannel endSwitchUp;
    DigitalChannel endSwitchDown;

    enum ManipulatorPosition {
        UP, DOWN, CENTRAL
    }

    private LinearOpMode linearOpMode;

    public Manipulator(LinearOpMode linearOpMode) {
        this.linearOpMode = linearOpMode;
    }

    private Servo liftMotor = null;

    public void init() {
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
