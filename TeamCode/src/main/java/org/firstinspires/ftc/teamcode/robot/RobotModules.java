package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Movement;

import java.util.Arrays;

public class RobotModules {
    private final LinearOpMode linearOpMode;
    public final Duck duck;
    public final Elevator elevator;
    public final Movement movement;
    public final Manipulator manipulator;

    private final RobotModule[] allModules;

    public RobotModules(LinearOpMode linearOpMode){
        this.linearOpMode = linearOpMode;
        duck = new Duck(linearOpMode);
        manipulator = new Manipulator(linearOpMode);
        elevator = new Elevator(linearOpMode);
        movement = new Movement(linearOpMode);

        allModules = new RobotModule[]{
                duck,
                elevator,
                movement,
                manipulator
        };
    }

    public void init(){
        for(RobotModule robotModule: allModules)
            robotModule.init();
    }
    public void update(){
        for(RobotModule robotModule: allModules)
            robotModule.update();
    }
    public boolean line(){
        return Arrays.stream(allModules).allMatch(robotModule -> robotModule.line() == true);
    }
}
