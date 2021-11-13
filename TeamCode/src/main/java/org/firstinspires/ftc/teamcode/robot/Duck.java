package org.firstinspires.ftc.teamcode.robot;


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
    }

    public void activity(boolean n){
        DuckSpin(n);
    }
    private boolean doSpin = false;
    public void DuckSpin(boolean ds){
        doSpin = ds;
    }
    public  void setDirection(int direction){
        this.direction = direction;
    }
    private int direction = 1;
    public boolean queuebool = true;
    public boolean line(){
        return queuebool;
    }
    public void update(){
        if(doSpin) {
            duckMotor.setPower(Range.clip(direction*.5,-1,1));
            queuebool = false;
        }
        else {
            queuebool = true;
            duckMotor.setPower(Range.clip(0,-1,1));
        }
    }

}
