package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Manipulator;
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
    }
}
