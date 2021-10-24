package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.teamcode.VariablesDashboard.ManipulatorConfig.manipulatorCentral;
import static org.firstinspires.ftc.teamcode.VariablesDashboard.ManipulatorConfig.manipulatorDown;
import static org.firstinspires.ftc.teamcode.VariablesDashboard.ManipulatorConfig.manipulatorUp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Movement;
import org.firstinspires.ftc.teamcode.opmodes.ButtonActivatedModes.Duck;

public class RobotModules {
    private final LinearOpMode linearOpMode;
    public final Duck duck;
    public final Movement movement;
    public final Manipulator manipulator;

    public RobotModules(LinearOpMode linearOpMode){
        this.linearOpMode = linearOpMode;
        duck = new Duck(linearOpMode);
        movement = new Movement(linearOpMode);
        manipulator = new Manipulator(linearOpMode);
    }

    public void init(){
        duck.init();
        movement.init();
        manipulator.init();
        movement.initGyro();
    }

    public static class Manipulator {

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
}
