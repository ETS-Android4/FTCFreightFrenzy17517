package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

public class AllFunction {
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotorEx leftMotorFront = null;
    private DcMotorEx rightMotorFront = null;
    private DcMotorEx leftMotorBack = null;
    private DcMotorEx rightMotorBack = null;

    /*
       private DcMotorEx fstv = null;
       private DcMotorEx EncX = null;
       private DcMotorEx EncY = null;
       private Servo finger = null;
       private final ElapsedTime timer1 = new ElapsedTime();
       private Servo fsrv = null;
       private Servo fcol = null;
   */
    public void motors(double y1, double x1) {
        double leftMotorFrontP = Range.clip(x1, -1.0, 1.0);
        double rightMotorFrontP = Range.clip(y1, -1.0, 1.0);
        leftMotorFront.setPower(leftMotorFrontP);
        rightMotorFront.setPower(rightMotorFrontP);
    }
}