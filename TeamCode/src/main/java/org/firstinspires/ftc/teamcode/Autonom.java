package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
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

@Autonomous
public class Autonom extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotorEx leftMotorFront = null;
    private DcMotorEx rightMotorFront = null;
    private DcMotorEx leftMotorBack = null;
    private DcMotorEx rightMotorBack = null;

    public void Angle(double angleF, char direction){
        double xR = rightMotorFront.getCurrentPosition();
        double yL = leftMotorFront.getCurrentPosition();
        double speed = 0.5;
        final double R = rightMotorFront.getCurrentPosition();
        final double L = leftMotorFront.getCurrentPosition();

        while((xR > (R+angleF) && direction == 'R') || (yL > (L+angleF) && direction =='L')){
            if (direction == 'R') {
                rightMotorFront.setPower(-speed);
                leftMotorFront.setPower(speed);
            }
            else if (direction == 'L') {
                rightMotorFront.setPower(speed);
                leftMotorFront.setPower(-speed);
            }
        }
    }

    public void Move(double distance, double angle, char direction) {
        double xR = rightMotorFront.getCurrentPosition();
        double yL = leftMotorFront.getCurrentPosition();
        double speed_R = 0;
        double speed_L = 0;
        double kf = 1;
        speed_R = (distance - xR) * kf;
        speed_L = (distance - yL) * kf;
        rightMotorFront.setPower(speed_R);
        leftMotorFront.setPower(speed_L);
        Angle(angle,direction);
    }

    public void Move(double dist){
        Move(dist, 0, 'o');
    }

    AllFunction fun = new AllFunction();

    @Override
    public void runOpMode() {

        rightMotorFront = hardwareMap.get(DcMotorEx.class, "R1");
        rightMotorBack = hardwareMap.get(DcMotorEx.class, "R2");
        leftMotorFront = hardwareMap.get(DcMotorEx.class, "L1");
        leftMotorBack = hardwareMap.get(DcMotorEx.class, "L2");

        rightMotorFront.setDirection(DcMotorEx.Direction.FORWARD);
        leftMotorFront.setDirection(DcMotorEx.Direction.REVERSE);
        rightMotorBack.setDirection(DcMotorEx.Direction.REVERSE);
        leftMotorBack.setDirection(DcMotorEx.Direction.FORWARD);

        waitForStart();

        telemetry.addData("Status", "Initialized");

        Move(0,200, 'R');
        Move(1000);
        telemetry.addData("Rrrrr", rightMotorFront.getCurrentPosition());
        telemetry.addData("Lllll", leftMotorFront.getCurrentPosition());
        telemetry.update();
    }
}