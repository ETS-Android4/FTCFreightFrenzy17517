package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

@TeleOp
public class TeleOP extends LinearOpMode {
    // Declare OpMode members.
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
    void motors (double y1, double x1) {
      double leftMotorFrontP = Range.clip(x1, -1.0, 1.0);
      double rightMotorFrontP = Range.clip(y1, -1.0, 1.0);
      leftMotorFront.setPower(leftMotorFrontP);
      rightMotorFront.setPower(rightMotorFrontP);
    }

    public void runOpMode(){
        waitForStart();
        rightMotorFront = hardwareMap.get(DcMotorEx.class, "R1");
        rightMotorBack = hardwareMap.get(DcMotorEx.class, "R2");
        leftMotorFront = hardwareMap.get(DcMotorEx.class, "L1");
        leftMotorBack = hardwareMap.get(DcMotorEx.class, "L2");

        rightMotorFront.setDirection(DcMotorEx.Direction.FORWARD);
        leftMotorFront.setDirection(DcMotorEx.Direction.REVERSE);
        rightMotorBack.setDirection(DcMotorEx.Direction.REVERSE);
        leftMotorBack.setDirection(DcMotorEx.Direction.FORWARD);

        /*fstv = hardwareMap.get(DcMotorEx.class, "Shooter");
        //Encoders
        EncX = hardwareMap.get(DcMotorEx.class, "EncY");
        EncY = hardwareMap.get(DcMotorEx.class, "EncX");
        //Servo
        fsrv = hardwareMap.get(Servo.class, "Fix");
        fcol = hardwareMap.get(Servo.class, "Col");
        finger = hardwareMap.get(Servo.class, "Finger");*/

 /*       fstv.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        fstv.setDirection(DcMotorSimple.Direction.FORWARD);
        EncX.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        EncY.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        EncX.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        EncY.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);*/
        runtime.reset();

        while (opModeIsActive()){
            rightMotorFront.setPower(gamepad1.left_stick_y-gamepad1.right_stick_x);
            leftMotorFront.setPower(gamepad1.left_stick_y+gamepad1.right_stick_x);
            telemetry.addData("encr", rightMotorFront.getCurrentPosition());
            telemetry.addData("encl", leftMotorFront.getCurrentPosition());
            telemetry.update();
        }
    }
}
