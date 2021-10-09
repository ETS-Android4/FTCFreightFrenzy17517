package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

@Autonomous
public class Autonom extends LinearOpMode {

    //creating variables
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotorEx leftMotorFront = null;
    private DcMotorEx rightMotorFront = null;
    private DcMotorEx leftMotorBack = null;
    private DcMotorEx rightMotorBack = null;

    public void Move(double distance, double angle) {
        double linerSpeed = 0;
        double currentAngle = 0;
        double integralSpeed = 0;
        double integralAngle = 0;
        double kP = 0.001;
        double kPA = 0.001;
        double kI = 0.00001;
        double kD = 0;
        do {
            //proportional component
            linerSpeed = (distance - (rightMotorFront.getCurrentPosition() + leftMotorFront.getCurrentPosition()) / 2.0) * kP;
            currentAngle = (angle - (rightMotorFront.getCurrentPosition() - leftMotorFront.getCurrentPosition())/2.0) * kPA;

            //integral component
            integralSpeed = (distance - (rightMotorFront.getCurrentPosition() + leftMotorFront.getCurrentPosition()) / 2.0) * kI;
            integralAngle = (angle - (rightMotorFront.getCurrentPosition() - leftMotorFront.getCurrentPosition())/2.0) * kI;

            //differential component


            rightMotorFront.setPower(linerSpeed + currentAngle + integralSpeed);
            leftMotorFront.setPower(linerSpeed - currentAngle + integralAngle);

            telemetry.addData("speed", linerSpeed);
            telemetry.addData("Rrrrr", rightMotorFront.getCurrentPosition());
            telemetry.addData("Lllll", leftMotorFront.getCurrentPosition());
            telemetry.update();
        } while(Math.abs(linerSpeed) > 0.05 || Math.abs(currentAngle) > 0.05);
   }

    public void Move(double dist){
        Move(dist, 0);
    }

    @Override
    public void runOpMode() {

        //init
        rightMotorFront = hardwareMap.get(DcMotorEx.class, "R1");
        rightMotorBack = hardwareMap.get(DcMotorEx.class, "R2");
        leftMotorFront = hardwareMap.get(DcMotorEx.class, "L1");
        leftMotorBack = hardwareMap.get(DcMotorEx.class, "L2");

        //set direction
        rightMotorFront.setDirection(DcMotorEx.Direction.FORWARD);
        leftMotorFront.setDirection(DcMotorEx.Direction.REVERSE);
        rightMotorBack.setDirection(DcMotorEx.Direction.REVERSE);
        leftMotorBack.setDirection(DcMotorEx.Direction.FORWARD);

        //zeroing encoders
        for(DcMotorEx dcMotorEx: hardwareMap.getAll(DcMotorEx.class)) {
            dcMotorEx.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            dcMotorEx.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        waitForStart();
        Move(0, 200);
    }
}