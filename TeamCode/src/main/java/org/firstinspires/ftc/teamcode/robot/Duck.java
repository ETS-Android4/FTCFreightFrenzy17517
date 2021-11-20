package org.firstinspires.ftc.teamcode.robot;


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

    private boolean doSpin = false;
    private double time = 1;
    public void DuckSpin(boolean ds){
        if(doSpin!=ds)duckTimer.reset();
        doSpin = ds;
    }
    public  void setDirection(int direction){
        this.direction = direction;
    }
    private int direction = 1;
    public boolean queuebool = true;
    private ElapsedTime duckTimer = new ElapsedTime();
    public boolean line(){
        return queuebool;
    }
    public void update(){
        if(doSpin && duckTimer.seconds()<time) {
            duckMotor.setPower(Range.clip(direction*.5,-1,1));
            queuebool = false;
        }
        else {
            queuebool = true;
            duckMotor.setPower(0);
        }
    }

}
