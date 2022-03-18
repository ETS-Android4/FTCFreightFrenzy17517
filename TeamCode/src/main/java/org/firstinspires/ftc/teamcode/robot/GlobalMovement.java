package org.firstinspires.ftc.teamcode.robot;

import static org.firstinspires.ftc.teamcode.robot.GlobalMovement.GMConfig.*;
import static org.firstinspires.ftc.teamcode.robot.Movement.MovementConfig.*;

import static java.lang.Math.PI;
import static java.lang.Math.abs;
import static java.lang.Math.cos;
import static java.lang.Math.signum;
import static java.lang.Math.sqrt;
import static java.lang.Math.atan2;
import static java.lang.Math.toRadians;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.misc.CommandSender;


public class GlobalMovement implements RobotModule {

    private static final double TRACK_WIDTH_CM = 38.84500;
    private static final double WHEEL_DIAMETER_CM = 10.90532;
    private static final int ENCODER_RESOLUTION = 24;
    private static final double GEARBOX_RATIO = 20.0;
    private static final double ENCODER_TICKS_TO_CM_RATIO =
            (WHEEL_DIAMETER_CM * PI) / (ENCODER_RESOLUTION * GEARBOX_RATIO);
    private static final double CM_TO_ROTATION_DEGREES_RATIO = 180.0 / ((TRACK_WIDTH_CM / 2.0) * PI);
    private final ElapsedTime loopTimer = new ElapsedTime();
    public boolean queuebool = true;
    ElapsedTime timer = new ElapsedTime();
    double oldErrDistance = 0;
    double oldErrAngle = 0;
    double proportionalLinear = 0;
    double proportionalAngular = 0;
    double integralLinear = 0;
    double integralAngular = 0;
    double differentialLinear = 0;
    double differentialAngular = 0;
    private boolean manualControl = true;
    private WoENRobot robot = null;
    private double distance = 0;
    private double angle = 0;
    private double speed = 0;
    private DcMotorEx leftMotorFront = null;
    private DcMotorEx leftMotorBack = null;
    private final CommandSender leftMotorCommandSender = new CommandSender((double value) -> {
        leftMotorBack.setPower(value);
        leftMotorFront.setPower(value);
    });
    private DcMotorEx rightMotorFront = null;
    private DcMotorEx rightMotorBack = null;
    private final CommandSender rightMotorCommandSender = new CommandSender((double value) -> {
        rightMotorBack.setPower(value);
        rightMotorFront.setPower(value);
    });

    public GlobalMovement(WoENRobot robot) {
        this.robot = robot;
    }

    public boolean actionIsCompleted() {
        return queuebool;
    }

    double getGyroHeading() {
        return -robot.gyro.getOrientation().firstAngle;
    }

    public double encoderTicksToRotationDegrees(double ticks) {
        return encoderTicksToCm(ticks) * CM_TO_ROTATION_DEGREES_RATIO;
    }

    public double cmToEncoderTicks(double centimeters) {
        return centimeters / ENCODER_TICKS_TO_CM_RATIO;
    }

    public double encoderTicksToCm(double ticks) {
        return ticks * ENCODER_TICKS_TO_CM_RATIO;
    }

    private void assignHardware() {
        rightMotorFront = robot.getLinearOpMode().hardwareMap.get(DcMotorEx.class, "R1");
        rightMotorBack = robot.getLinearOpMode().hardwareMap.get(DcMotorEx.class, "R2");
        leftMotorFront = robot.getLinearOpMode().hardwareMap.get(DcMotorEx.class, "L1");
        leftMotorBack = robot.getLinearOpMode().hardwareMap.get(DcMotorEx.class, "L2");
    }

    private void setDirections() {
        rightMotorFront.setDirection(DcMotorEx.Direction.FORWARD);
        leftMotorFront.setDirection(DcMotorEx.Direction.REVERSE);
        rightMotorBack.setDirection(DcMotorEx.Direction.FORWARD);
        leftMotorBack.setDirection(DcMotorEx.Direction.REVERSE);
    }

    public double getCurrent() {
        return leftMotorFront.getCurrent(CurrentUnit.AMPS) + leftMotorBack.getCurrent(CurrentUnit.AMPS) +
                rightMotorFront.getCurrent(CurrentUnit.AMPS) + rightMotorBack.getCurrent(CurrentUnit.AMPS);
    }

    public void resetEncoders(DcMotor.RunMode runMode) {
        leftMotorFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotorBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotorFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotorBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotorFront.setMode(runMode);
        leftMotorBack.setMode(runMode);
        rightMotorFront.setMode(runMode);
        rightMotorBack.setMode(runMode);
    }

    private void setZeroPowerBehaviors(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        leftMotorFront.setZeroPowerBehavior(zeroPowerBehavior);
        leftMotorBack.setZeroPowerBehavior(zeroPowerBehavior);
        rightMotorFront.setZeroPowerBehavior(zeroPowerBehavior);
        rightMotorBack.setZeroPowerBehavior(zeroPowerBehavior);
    }

    public void initialize() {
        assignHardware();
        setDirections();
        resetEncoders(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        setZeroPowerBehaviors(zeroPowerBehavior);
    }

    public double xError = 0;
    public double yError = 0;

    public double getDistanceError() {
        xError = xTarget - robot.odometry.getCurrentPosition().x;
        yError = yTarget - robot.odometry.getCurrentPosition().y;
        distance = sqrt(xError * xError + yError * yError);
       // if (xErrorForTan < 0 - MinErrorForX) return -distance;
        /*else*/ return distance;
    }

    public double getAngleError() {
        double error = -atan2(yError, xError) - robot.odometry.getCurrentPosition().heading;
        //if (xErrorForTan < 0 - MinErrorForX) error = -atan2( -yError, -xError) - robot.odometry.getCurrentPosition().heading;
        if (error > PI) do error -= PI*2; while (error > PI);
        else if (error < -PI) do error += PI*2; while (error < -PI);
        return error;
    }

    public void Move(double x, double y, double a) {
        Move(x, y, a,1);
    }

    public void Move(double x, double y) {
        Move(x, y, Double.NaN,1);
    }

    private double xTarget = 0;
    private double yTarget = 0;
    private double angleTarget = 0;
    //private double xErrorForTan = 0;
    public void Move(double xT, double yT, double aT, double speed) {
        timer.reset();
        xTarget = xT;
        yTarget = yT;
        angleTarget = aT;
        manualControl = false;
        queuebool = false;
        if (distance != this.distance || angle != this.angle || speed != this.speed) {
            loopTimer.reset();
            oldErrDistance = getDistanceError();
            oldErrAngle = getAngleError();
        }
        //xErrorForTan = xT - robot.odometry.getCurrentPosition().x;
        this.speed = speed;
    }


    public void update() {
        if (!manualControl) {
            boolean inRadius = false;
            double deltaErrDistance = 0;
            double deltaErrAngle = 0;
            double speedAngle = 1;
            double errDistance = getDistanceError();
            double errAngle = getAngleError();
            if(errDistance < MinErrorDistance && !Double.isNaN(angleTarget)) inRadius = true;
            if(errDistance > MinErrorDistance * 2) inRadius = false;
            errDistance *= cos(errAngle);
            if (errAngle > PI/2) do errAngle -= PI; while (errAngle > PI/2);
            else if (errAngle < -PI/2) do errAngle += PI; while (errAngle < -PI/2);
            if (inRadius){
                errAngle = angleTarget - robot.odometry.getCurrentPosition().heading;
                if (errAngle > PI) do errAngle -= PI*2; while (errAngle > PI);
                else if (errAngle < -PI) do errAngle += PI*2; while (errAngle < -PI);
            }
            double timestep = loopTimer.seconds();
            loopTimer.reset();
            {   //proportional component
                proportionalLinear = errDistance * G_kP_Distance;
                proportionalAngular = errAngle * G_kP_Angle;
            }
            {   //integral component  (|0.25|)
                integralLinear += errDistance * G_kI_Distance * timestep;
                integralAngular += errAngle * G_kI_Angle * timestep;
                if (abs(integralAngular) > maxIntegralAngle)
                    integralAngular = maxIntegralAngle * signum(integralAngular);
                if (abs(integralLinear) > maxIntegralDistance)
                    integralLinear = maxIntegralDistance * signum(integralLinear);
            }
            {   //differential component
                deltaErrDistance = errDistance - oldErrDistance;
                deltaErrAngle = errAngle - oldErrAngle;
                oldErrDistance = errDistance;
                oldErrAngle = errAngle;
                differentialLinear = (deltaErrDistance / timestep) * G_kD_Distance;
                differentialAngular = (deltaErrAngle / timestep) * G_kD_Angle;
            }
            setMotorPowersPrivate((integralLinear + proportionalLinear + differentialLinear) * speed * robot.battery.getkVoltage(),
                    (integralAngular + proportionalAngular + differentialAngular) * speedAngle *
                            robot.battery.getkVoltage());
            queuebool = ((abs(errDistance) < minErrorDistance - 1) && (errAngle < toRadians(2))) /*||
                    (timer.seconds() >= moveTimeoutS)*/;
        } else {
            queuebool = true;
        }

    }

    public void telemetryForMovement(Telemetry telemetry) {
        telemetry.addData("Proportional(linear)", proportionalLinear);
        telemetry.addData("Proportional(angle)", proportionalAngular);
        telemetry.addData("Differential(linear)", differentialLinear);
        telemetry.addData("Differential(angle)", differentialAngular);
        telemetry.addData("Integral(linear)", integralLinear);
        telemetry.addData("Integral(angle)", integralAngular);
        telemetry.addData("angleErr", getAngleError());
        telemetry.addData("ErrDistance", getDistanceError());
        telemetry.addData("arctg", -atan2(yError,xError));
        telemetry.addData("arctgFor-", -atan2(-yError,-xError));
        telemetry.addData("ErrDistance * cos", getDistanceError()* cos(getAngleError()));
    }

    public void setMotorPowers(double power, double angle) {
        manualControl = true;
        setMotorPowersPrivate(power, angle);
    }

    private void setMotorPowersPrivate(double power, double angle) {
        rightMotorCommandSender.send(Range.clip(power - angle, -MaxSpeed, MaxSpeed));
        leftMotorCommandSender.send(Range.clip(power + angle, -MaxSpeed, MaxSpeed));
    }

    double getLeftEncoder() {
        return (leftMotorBack.getCurrentPosition() + leftMotorFront.getCurrentPosition()) / 2.0;
    }

    double getRightEncoder() {
        return (rightMotorFront.getCurrentPosition() + rightMotorBack.getCurrentPosition()) / 2.0;
    }

    @Config
    public static class GMConfig {
        public static double G_kP_Distance = 0.02;
        public static double G_kP_Angle = 5;
        public static double G_kI_Distance = 0.01;
        public static double G_kI_Angle = 0.02;
        public static double G_kD_Distance = 0.01;
        public static double G_kD_Angle = 0.02;
        public static double MaxSpeed = 1;
        public static double MinErrorDistance = 3;
    }
}