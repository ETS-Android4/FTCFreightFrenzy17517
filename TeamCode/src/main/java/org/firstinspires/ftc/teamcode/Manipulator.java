package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.VariablesDash.*;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class Manipulator {
    private LinearOpMode linearOpMode = null;
    public Manipulator(LinearOpMode linearOpMode){
        this.linearOpMode = linearOpMode;
    }

    private DcMotorEx liftMotor = null;
    void initManip(){
        liftMotor = linearOpMode.hardwareMap.get(DcMotorEx.class, "UpDown");

        for(DcMotorEx dcMotorEx: linearOpMode.hardwareMap.getAll(DcMotorEx.class)) {
            dcMotorEx.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            dcMotorEx.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }
}
