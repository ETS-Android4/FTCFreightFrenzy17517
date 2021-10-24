package org.firstinspires.ftc.teamcode.opmodes.ButtonActivatedModes;


import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public class Duck implements ButtonActivated {
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

    public static int direction = 1;

    public void DuckSpin(boolean doSpin){
        if(doSpin) {
            duckMotor.setPower(Range.clip(direction*.6,-1,1));

        }
        else duckMotor.setPower(Range.clip(0,-1,1));
    }
    public void setDirection(int direction){
        Duck.direction = direction;
    }
}
