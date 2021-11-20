package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.Arrays;

public class RobotModules {
    private static LinearOpMode linearOpMode;
    public static Duck duck;
    public static Elevator elevator;
    public static Movement movement;
    public static Arsen intake;

    private final RobotModule[] allModules;

    public RobotModules(LinearOpMode linearOpMode){
        RobotModules.linearOpMode = linearOpMode;
        duck = new Duck(linearOpMode);
        elevator = new Elevator(linearOpMode);
        movement = new Movement(linearOpMode);
        intake = new Arsen(linearOpMode);

        allModules = new RobotModule[]{
                duck,
                elevator,
                movement,
                intake
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
        return Arrays.stream(allModules).allMatch(RobotModule::line);
    }
}
