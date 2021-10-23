package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.VariablesDash.*;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

public class Manipulator {
    DigitalChannel digitalTouch;

    private LinearOpMode linearOpMode = null;
    public Manipulator(LinearOpMode linearOpMode){
        this.linearOpMode = linearOpMode;
    }

    private Servo liftMotor = null;
    public void initManip() {
        liftMotor = linearOpMode.hardwareMap.get(Servo.class, "UpDown");
    }
    public void MoveServo (double position){
        liftMotor.setPosition(position);
    }
}
