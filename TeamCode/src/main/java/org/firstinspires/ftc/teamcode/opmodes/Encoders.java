package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public class Encoders {
    private LinearOpMode linearOpMode = null;
    private DcMotor R1 = null;
    private DcMotor L1 = null;
    private int angle;
    private double i;

    public Encoders(LinearOpMode linearOpMode) {
        this.linearOpMode = linearOpMode;
    }

    public void init() {
        R1 = linearOpMode.hardwareMap.get(DcMotor.class, "R1");
        L1 = linearOpMode.hardwareMap.get(DcMotor.class, "L1");
    }

    public int Get_1() {
        return R1.getCurrentPosition();
    }

    public int Get_2() {
        return L1.getCurrentPosition();
    }

    public int get_angle() {
        angle = (Get_2() - Get_1()) / (480);
        i = angle / 360;
        if (angle < 0) {
            angle += i * 360;
        } else {
            angle -= i * 360;
        }
        return angle;
    }
}