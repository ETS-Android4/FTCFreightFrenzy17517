package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.VariablesDash.*;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.bosch.BNO055IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;


public class Movement {
    BNO055IMU gyro = null;

    void initGyro() {
        gyro = linearOpMode.hardwareMap .get(BNO055IMU.class, "imu");
        gyro.initialize(new BNO055IMU.Parameters());
    }
    private LinearOpMode linearOpMode = null;

    public Movement(LinearOpMode linearOpMode){
        this.linearOpMode = linearOpMode;
    }

    double getGyroHeading() {
        return gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
    }


    private ElapsedTime runtime = new ElapsedTime();
    private DcMotorEx leftMotorFront = null;
    private DcMotorEx rightMotorFront = null;
    private DcMotorEx leftMotorBack = null;
    private DcMotorEx rightMotorBack = null;
    public double smToInc (double getCurrentPosition){
        return getCurrentPosition/14.42;
    }
    public void init(){
        //init

        rightMotorFront = linearOpMode.hardwareMap.get(DcMotorEx.class, "R1");
        rightMotorBack = linearOpMode.hardwareMap.get(DcMotorEx.class, "R2");
        leftMotorFront = linearOpMode.hardwareMap.get(DcMotorEx.class, "L1");
        leftMotorBack = linearOpMode.hardwareMap.get(DcMotorEx.class, "L2");

        //set direction
        rightMotorFront.setDirection(DcMotorEx.Direction.REVERSE);
        leftMotorFront.setDirection(DcMotorEx.Direction.FORWARD);
        rightMotorBack.setDirection(DcMotorEx.Direction.REVERSE);
        leftMotorBack.setDirection(DcMotorEx.Direction.FORWARD);

        //zeroing encoders
        for(DcMotorEx dcMotorEx: linearOpMode.hardwareMap.getAll(DcMotorEx.class)) {
            dcMotorEx.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            dcMotorEx.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        initGyro();

    }

    public void Move(double distance, double angle) {
        double linerSpeed = 0;
        double currentAngle = 0;
        double integralSpeed = 0;
        double integralAngle = 0;
        double errDistance = 0;
        double oldErrDistance = (distance - (Encoder("right") + Encoder("left")) / 2.0);
        double oldErrAngle = (angle - Encoder("right") - Encoder("left"));
        double errAngle = 0;
        double difSpeed = 0;
        double difAngle = 0;
        runtime.reset();
        do {
            errDistance = (distance - (smToInc(Encoder("right")) + smToInc(Encoder("left")))/ 2.0);
            errAngle = (angle + smToInc(Encoder("right")) - smToInc(Encoder("left")));
            errAngle = (angle - getGyroHeading());
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

            Motor(integralSpeed+linerSpeed+difSpeed, -integralAngle - currentAngle - difAngle);

            Telemetry currentTelemetry;
           // currentTelemetry = linearOpMode.telemetry;
            currentTelemetry = FtcDashboard.getInstance().getTelemetry();

            currentTelemetry.addData("sec", runtime.seconds());
            currentTelemetry.addData("liner speed", linerSpeed);
            currentTelemetry.addData("current angle", currentAngle);
            currentTelemetry.addData("integral speed", integralSpeed);
            currentTelemetry.addData("integral angle", integralAngle);
            currentTelemetry.addData("difSpeed",difSpeed);
            currentTelemetry.addData("difAngle",difAngle);
            currentTelemetry.addData("PowerRight",rightMotorFront.getPower());
            currentTelemetry.addData("PowerLeft",leftMotorFront.getPower());
            currentTelemetry.addData("ErrAngle", errAngle);
            currentTelemetry.addData("Rrrrr", smToInc(Encoder("right")));
            currentTelemetry.addData("Lllll", smToInc(Encoder("left")));
            FtcDashboard.getInstance().getTelemetry().update();

            runtime.reset();

        } while((Math.abs(errDistance) > 5 || Math.abs(errAngle) > 5) && linearOpMode.opModeIsActive());
    }
    public void Motor (double power, double angle){
        rightMotorFront.setPower(power - angle);
        rightMotorBack.setPower(power - angle);
        leftMotorFront.setPower(power + angle);
        leftMotorBack.setPower(power + angle);
    }
    double Encoder(String direction) {
        double Enc = 0;
        if (direction == "left") {
            Enc = (leftMotorBack.getCurrentPosition() + leftMotorFront.getCurrentPosition())/2.0;
        } else {
            Enc = (rightMotorFront.getCurrentPosition()+rightMotorBack.getCurrentPosition())/2.0;
        }
    return Enc;
    }

    public void Move(double dist){
        Move(dist, 0);
    }

}
