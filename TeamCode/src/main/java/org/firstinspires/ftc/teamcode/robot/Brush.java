package org.firstinspires.ftc.teamcode.robot;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.opmodes.TeleOpOneGamepad;


public class Brush implements RobotModule {
    private DcMotorEx brushMotor = null;
    private LinearOpMode linearOpMode = null;
    public DistanceSensor distance = null;
    public DcMotorEx ledMotor = null;
    public boolean freightIsDetected = false;


    public Brush(LinearOpMode linearOpMode) {
        this.linearOpMode = linearOpMode;
    }

    public void init() {
        brushMotor = linearOpMode.hardwareMap.get(DcMotorEx.class, "BrushMotor");
        brushMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        brushMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        brushMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        brushMotor.setDirection(DcMotorEx.Direction.REVERSE);
        distance = linearOpMode.hardwareMap.get(DistanceSensor.class, "distance");
        ledMotor = linearOpMode.hardwareMap.get(DcMotorEx.class, "Led");

    }

    public boolean intake = false;

    public void brushMotorMove(boolean intake) {
        this.intake = intake;
    }

    public void update() {
        boolean freightIsDetected = distance.getDistance(DistanceUnit.CM) < 8;
        double brushPower = freightIsDetected ? -1 : 1;
        brushMotor.setPower(intake ? brushPower : 0);
        ledMotor.setPower(freightIsDetected ? 1 : 0);
    }

    public boolean line() {
        return true;
    }
}
