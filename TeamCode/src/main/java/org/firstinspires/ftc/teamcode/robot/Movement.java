package org.firstinspires.ftc.teamcode.robot;

import static org.firstinspires.ftc.teamcode.robot.Movement.MovementConfig.kD_Angle;
import static org.firstinspires.ftc.teamcode.robot.Movement.MovementConfig.kD_Distance;
import static org.firstinspires.ftc.teamcode.robot.Movement.MovementConfig.kI_Angle;
import static org.firstinspires.ftc.teamcode.robot.Movement.MovementConfig.kI_Distance;
import static org.firstinspires.ftc.teamcode.robot.Movement.MovementConfig.kP_Angle;
import static org.firstinspires.ftc.teamcode.robot.Movement.MovementConfig.kP_Distance;
import static org.firstinspires.ftc.teamcode.robot.Movement.MovementConfig.maxIntegralAngle;
import static org.firstinspires.ftc.teamcode.robot.Movement.MovementConfig.maxIntegralDistance;
import static org.firstinspires.ftc.teamcode.robot.Movement.MovementConfig.minErrorAngle;
import static org.firstinspires.ftc.teamcode.robot.Movement.MovementConfig.minErrorDistance;
import static org.firstinspires.ftc.teamcode.robot.Movement.MovementConfig.moveTimeoutS;
import static org.firstinspires.ftc.teamcode.robot.Movement.MovementConfig.zeroPowerBehavior;

import static java.lang.Math.PI;
import static java.lang.Math.abs;
import static java.lang.Math.signum;
import static java.lang.Math.sqrt;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.misc.CommandSender;


public class Movement implements RobotModule {

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

    public Movement(WoENRobot robot) {
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

    public double getCurrent(){
        return leftMotorFront.getCurrent(CurrentUnit.AMPS) + leftMotorBack.getCurrent(CurrentUnit.AMPS) + rightMotorFront.getCurrent(CurrentUnit.AMPS) + rightMotorBack.getCurrent(CurrentUnit.AMPS);
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

    public double getDistanceError(double target) {
        return target - encoderTicksToCm((getLeftEncoder() + getRightEncoder()) / 2.0);
    }

    public double getAngleError(double target) {
        double error = target - getGyroHeading();
        if (error > 180.0) do error -= 360.0; while (error > 180.0);
        else if (error < -180.0) do error += 360.0; while (error < -180.0);
        return error;
    }

    public void Move(double distance) {
        Move(distance, 0, 1);
    }

    public void Move(double distance, double angle) {
        Move(distance, angle, 1);
    }

    public void Move(double distance, double angle, double speed) {
        timer.reset();
        manualControl = false;
        if (distance != this.distance || angle != this.angle || speed != this.speed) {
            loopTimer.reset();
            oldErrDistance = getDistanceError(this.distance);
            oldErrAngle = getAngleError(this.angle);
        }
        this.distance = distance;
        this.angle = angle;
        this.speed = speed;
    }


    public void update() {
        if (!manualControl) {
            double deltaErrDistance = 0;
            double deltaErrAngle = 0;
            double speedAngle = 1;
            double errDistance = getDistanceError(distance);
            double errAngle = getAngleError(angle);
            double timestep = loopTimer.seconds();
            loopTimer.reset();
            {   //proportional component
                proportionalLinear = errDistance * kP_Distance;
                proportionalAngular = errAngle * kP_Angle;
            }
            {   //integral component  (|0.25|)
                integralLinear += errDistance * kI_Distance * timestep;
                integralAngular += errAngle * kI_Angle * timestep;
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
                differentialLinear = (deltaErrDistance / timestep) * kD_Distance;
                differentialAngular = (deltaErrAngle / timestep) * kD_Angle;
            }
            setMotorPowersPrivate((integralLinear + proportionalLinear + differentialLinear) * speed *
                            robot.battery.getkVoltage(),
                    (integralAngular + proportionalAngular + differentialAngular) * speedAngle *
                            robot.battery.getkVoltage());
            queuebool = ((abs(errDistance) < minErrorDistance) && (abs(errAngle) < minErrorAngle)) ||
                    (timer.seconds() >= moveTimeoutS);
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
    }

    public void setMotorPowers(double power, double angle) {
        manualControl = true;
        setMotorPowersPrivate(power, angle);
    }

    private void setMotorPowersPrivate(double power, double angle) {
        rightMotorCommandSender.send(power - angle);
        leftMotorCommandSender.send(power + angle);
    }

    double getLeftEncoder() {
        return (leftMotorBack.getCurrentPosition() + leftMotorFront.getCurrentPosition()) / 2.0;
    }

    double getRightEncoder() {
        return rightMotorFront.getCurrentPosition();
        // (rightMotorFront.getCurrentPosition() + rightMotorBack.getCurrentPosition()) / 2.0; //TODO fix wiring
    }

    @Config
    public static class MovementConfig {
        public static double kP_Distance = 0.02;
        public static double kP_Angle = 0.063;
        public static double kI_Distance = 0.004;
        public static double kI_Angle = 0.01;
        public static double kD_Distance = 0.002;
        public static double kD_Angle = 0.003;
        public static double maxIntegralAngle = 0.25;
        public static double maxIntegralDistance = 0.25;
        public static double minErrorDistance = 2.0;
        public static double minErrorAngle = 1;
        public static double moveTimeoutS = 2;
        public static DcMotor.ZeroPowerBehavior zeroPowerBehavior = DcMotor.ZeroPowerBehavior.FLOAT;
    }
}