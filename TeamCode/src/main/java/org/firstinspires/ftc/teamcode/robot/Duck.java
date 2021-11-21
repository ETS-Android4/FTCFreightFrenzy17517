package org.firstinspires.ftc.teamcode.robot;


import static org.firstinspires.ftc.teamcode.VariablesDashboard.Elevator.downTargetElevator;
import static org.firstinspires.ftc.teamcode.VariablesDashboard.ManipulatorConfig.*;
import static org.firstinspires.ftc.teamcode.VariablesDashboard.Duck.*;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public class Duck implements RobotModule {
    private DcMotor duckMotor = null;
    private LinearOpMode linearOpMode = null;

    public Duck(LinearOpMode linearOpMode){
        this.linearOpMode = linearOpMode;
    }

    public void init(){
        duckMotor = linearOpMode.hardwareMap.get(DcMotor.class, "DuckMotor");
        duckMotor.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public enum PositionOnField{
        RED, BLUE
    }
    public double direction = 1;
    public void redOrBlue(Duck.PositionOnField direction){
        switch (direction){
            case RED:
              directionDuck = 1;
            break;
            case BLUE:
              directionDuck = -1;
            break;
        }

    }
    private boolean doSpin = false;
    private double time = 5;
    public void Teleop(){
        time = 1.9;
        positonServoForElevator = 0.6;
    }
    public void DuckSpin(boolean ds){
        if(doSpin!=ds)duckTimer.reset();
        doSpin = ds;
    }
    public  void setDirection(int direction){
        this.direction = direction;
    }
    public boolean queuebool = true;
    private ElapsedTime duckTimer = new ElapsedTime();
    public boolean line(){
        return queuebool;
    }
    public void update(){
        if(doSpin && duckTimer.seconds()<time) {

            duckMotor.setPower(Range.clip(directionDuck*0.555,-1,1));
            queuebool = false;
        }
        else {
            queuebool = true;
            duckMotor.setPower(0);
        }
    }

}
