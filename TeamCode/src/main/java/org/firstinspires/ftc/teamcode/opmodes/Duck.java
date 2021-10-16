package org.firstinspires.ftc.teamcode.opmodes;


import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

public class Duck {
    private DcMotor duckMotor = null;
    private LinearOpMode linearOpMode = null;
    private Gamepad gamepad1 = linearOpMode.gamepad1;
    private int i = 0;

    public Duck(LinearOpMode linearOpMode){
        this.linearOpMode = linearOpMode;
    }

    public void init(){
        duckMotor = linearOpMode.hardwareMap.get(DcMotor.class, "DuckMotor");
    }

    public void spin(boolean doSpin){
        if(doSpin) duckMotor.setPower(Range.clip(1,-1,1));
        else duckMotor.setPower(Range.clip(0,-1,1));
    }
}
