package org.firstinspires.ftc.teamcode;


import static org.firstinspires.ftc.teamcode.VariablesDash.*;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Movement {

    private LinearOpMode linearOpMode = null;

    public Movement(LinearOpMode linearOpMode){
        this.linearOpMode = linearOpMode;
    }


    private ElapsedTime runtime = new ElapsedTime();
    private DcMotorEx leftMotorFront = null;
    private DcMotorEx rightMotorFront = null;
    private DcMotorEx leftMotorBack = null;
    private DcMotorEx rightMotorBack = null;

    public void init(){
        //init
        rightMotorFront = linearOpMode.hardwareMap.get(DcMotorEx.class, "R1");
        rightMotorBack = linearOpMode.hardwareMap.get(DcMotorEx.class, "R2");
        leftMotorFront = linearOpMode.hardwareMap.get(DcMotorEx.class, "L1");
        leftMotorBack = linearOpMode.hardwareMap.get(DcMotorEx.class, "L2");

        //set direction
        rightMotorFront.setDirection(DcMotorEx.Direction.FORWARD);
        leftMotorFront.setDirection(DcMotorEx.Direction.REVERSE);
        rightMotorBack.setDirection(DcMotorEx.Direction.REVERSE);
        leftMotorBack.setDirection(DcMotorEx.Direction.FORWARD);

        //zeroing encoders
        for(DcMotorEx dcMotorEx: linearOpMode.hardwareMap.getAll(DcMotorEx.class)) {
            dcMotorEx.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            dcMotorEx.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }


    public void Move(double distance, double angle) {
        double linerSpeed = 0;
        double currentAngle = 0;
        double integralSpeed = 0;
        double integralAngle = 0;
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
            oldErrAngle = errAngle;

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

            linearOpMode.telemetry.addData("sec", runtime.seconds());
            linearOpMode.telemetry.addData("liner speed", linerSpeed);
            linearOpMode.telemetry.addData("current angle", currentAngle);
            linearOpMode.telemetry.addData("integral speed", integralSpeed);
            linearOpMode.telemetry.addData("integral angle", integralAngle);
            linearOpMode.telemetry.addData("difSpeed",difSpeed);
            linearOpMode.telemetry.addData("difAngle",difAngle);
            linearOpMode.telemetry.addData("PowerRight",rightMotorFront.getPower());
            linearOpMode.telemetry.addData("PowerLeft",leftMotorFront.getPower());
            linearOpMode.telemetry.addData("ErrAngle", errAngle);
            linearOpMode.telemetry.addData("Rrrrr", rightMotorFront.getCurrentPosition());
            linearOpMode.telemetry.addData("Lllll", leftMotorFront.getCurrentPosition());
            linearOpMode.telemetry.update();

            runtime.reset();

        } while((Math.abs(errDistance) > 100 || Math.abs(errAngle) > 100) && linearOpMode.opModeIsActive());
    }

    public void Move(double dist){
        Move(dist, 0);
    }

}
