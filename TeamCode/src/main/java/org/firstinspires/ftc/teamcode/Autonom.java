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
        double kPA = 0.0015;
        double kI = 0.00001;
        double kIA = 0.0002;
        double kD = 0.0001;
        double kDA = 0.00001;
        double errDistance = 0;
        double oldErrDistance = (distance - (rightMotorFront.getCurrentPosition() + leftMotorFront.getCurrentPosition()) / 2.0);
        double oldErrAngle = (angle - rightMotorFront.getCurrentPosition() - leftMotorFront.getCurrentPosition());
        double errAngle = 0;
        double difSpeed = 0;
        double difAngle = 0;
        runtime.reset();
        do {
            errDistance = (distance - (rightMotorFront.getCurrentPosition() + leftMotorFront.getCurrentPosition()) / 2.0);
            errAngle = (angle - rightMotorFront.getCurrentPosition() - leftMotorFront.getCurrentPosition());
            double deltaErrDistance = errDistance - oldErrDistance;
            double deltaErrAngle = errAngle - oldErrAngle;
            oldErrDistance = errDistance;

            //proportional component
            linerSpeed = errDistance * kP;
            currentAngle = errAngle * kPA;

            //integral component  (|0.25|)
            integralSpeed += errDistance * kI * runtime.seconds();
            integralAngle += errAngle * kIA * runtime.seconds();
            if (integralAngle > 0.25) integralAngle = 0.25;
            if (integralSpeed > 0.25) integralSpeed = 0.25;
            if (integralAngle < -0.25) integralAngle = -0.25;
            if (integralSpeed < -0.25) integralSpeed = -0.25;

            //differential component
            difSpeed = deltaErrDistance/runtime.seconds() * kD;
            difAngle = deltaErrAngle/runtime.seconds() * kDA;


            rightMotorFront.setPower(integralSpeed + linerSpeed + difSpeed + integralAngle + currentAngle + difAngle);
            leftMotorFront.setPower(integralSpeed + linerSpeed + difSpeed - integralAngle - currentAngle - difAngle);

            telemetry.addData("sec", runtime.seconds());
            telemetry.addData("liner speed", linerSpeed);
            telemetry.addData("current angle", currentAngle);
            telemetry.addData("integral speed", integralSpeed);
            telemetry.addData("integral angle", integralAngle);
            telemetry.addData("difSpeed",difSpeed);
            telemetry.addData("difAngle",difAngle);
            telemetry.addData("PowerRight",rightMotorFront.getPower());
            telemetry.addData("PowerLeft",leftMotorFront.getPower());
            telemetry.addData("ErrAngle", errAngle);
            telemetry.addData("Rrrrr", rightMotorFront.getCurrentPosition());
            telemetry.addData("Lllll", leftMotorFront.getCurrentPosition());
            telemetry.update();

            runtime.reset();

        } while((Math.abs(errDistance) > 100 || Math.abs(errAngle) > 100) && opModeIsActive());
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
        Move(0,1000);

    }
}