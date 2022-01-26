package org.firstinspires.ftc.teamcode.robot;

import static org.firstinspires.ftc.teamcode.VariablesDashboard.MovementConfig.*;
import static java.lang.Math.PI;
import static java.lang.Math.abs;
import static java.lang.Math.signum;

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
import org.firstinspires.ftc.teamcode.misc.TimedSensorQuery;
import org.firstinspires.ftc.teamcode.robot.RobotModule;


public class Movement implements RobotModule {
    private BNO055IMU gyro = null;

    private TimedSensorQuery timedGyroQuery = new TimedSensorQuery(()->gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle, 1000);

    public boolean actionIsCompleted() {
        return queuebool;
    }

    public void initGyro() {
        gyro = robot.getLinearOpMode().hardwareMap.get(BNO055IMU.class, "imu");
        gyro.initialize(new BNO055IMU.Parameters());
    }


    private boolean manualControl = true;

    private WoENRobot robot = null;

    public Movement(WoENRobot robot) {
        this.robot = robot;
    }

    double getGyroHeading() {
        return -timedGyroQuery.getValue();
    }


    private double distance = 0;
    private double angle = 0;
    private double speed = 0;
    private final ElapsedTime runtime = new ElapsedTime();
    private DcMotorEx leftMotorFront = null;
    private DcMotorEx rightMotorFront = null;
    private DcMotorEx leftMotorBack = null;
    private DcMotorEx rightMotorBack = null;

    private static final double WHEEL_DIAMETER_CM = 10.6;
    private static final int ENCODER_RESOLUTION = 24;
    private static final double GEARBOX_RATIO = 20.0;
    private static final double ENCODER_TICKS_TO_CM_RATIO = (WHEEL_DIAMETER_CM * PI) / (ENCODER_RESOLUTION * GEARBOX_RATIO);

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
        initGyro();
        assignHardware();
        setDirections();
        resetEncoders(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        setZeroPowerBehaviors(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    public double getDistanceError(double target) {
        return target - encoderTicksToCm((getLeftEncoder() + getRightEncoder()) / 2.0);
    }

    public double getAngleError(double target) {
        double error = target - getGyroHeading(); //(angle + cmToEncoderTicks(getRightEncoder()) - cmToEncoderTicks(getLeftEncoder()));
        if (error > 180.0)
            do error -= 360.0; while (error > 180.0);
        else if (error < -180.0)
            do error += 360.0; while (error < -180.0);
        return error;
    }

    public boolean queuebool = true;
    public double timerForMovement = 2;

    public void forTeleOp() {
        timerForMovement = 0;
    }

    ElapsedTime timer = new ElapsedTime();

    public void Move(double di, double ag, double sp) {
        manualControl = false;
        if(di != distance || ag != angle || sp != speed)
            timer.reset();
        distance = di;
        angle = ag;
        speed = sp;
    }

    double oldErrDistance = 0;
    double oldErrAngle = 0;
    public void update() {
        double timestep = runtime.seconds();
        runtime.reset();
        double proportionalLinear = 0;
        double proportionalAngular = 0;
        double integralLinear = 0;
        double integralAngular = 0;
        double differentialLinear = 0;
        double differentialAngular = 0;
        double deltaErrDistance = 0;
        double deltaErrAngle = 0;
        double speedAngle = 1;
        double errDistance = getDistanceError(distance);
        double errAngle = getAngleError(angle);
        runtime.reset();
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
        if (!manualControl)
            setMotorPowersPrivate((integralLinear + proportionalLinear + differentialLinear) * speed * robot.accumulator.getkVoltage(),
                    (integralAngular + proportionalAngular + differentialAngular) * speedAngle  /* robot.accumulator.getkVoltage()*/);
        queuebool = (!(abs(errDistance) > minErrorDistance) && !(abs(errAngle) > minErrorAngle)) || (timer.seconds() >= timerForMovement);
        FtcDashboard.getInstance().getTelemetry().addData("error dist", errDistance);
        FtcDashboard.getInstance().getTelemetry().addData("timestep", timestep);
        FtcDashboard.getInstance().getTelemetry().addData("kvoltage", robot.accumulator.getkVoltage());
        FtcDashboard.getInstance().getTelemetry().update();
    }

    public void setMotorPowers(double power, double angle) {
        manualControl = true;
        setMotorPowersPrivate(power, angle);
    }
    public void teleometryEncoder(){
        FtcDashboard.getInstance().getTelemetry().addData("rightMotorFront",rightMotorFront.getCurrentPosition());
        FtcDashboard.getInstance().getTelemetry().addData("rightMotorBack",rightMotorBack.getCurrentPosition());
        FtcDashboard.getInstance().getTelemetry().addData("leftMotorFront",leftMotorFront.getCurrentPosition());
        FtcDashboard.getInstance().getTelemetry().addData("leftMotorBack",leftMotorBack.getCurrentPosition());
        FtcDashboard.getInstance().getTelemetry().update();
    }

    private void setMotorPowersPrivate(double power, double angle) {
        rightMotorFront.setPower(power - angle);
        rightMotorBack.setPower(power - angle);
        leftMotorFront.setPower(power + angle);
        leftMotorBack.setPower(power + angle);
    }

    double getLeftEncoder() {
        return (leftMotorBack.getCurrentPosition() + leftMotorFront.getCurrentPosition()) / 2.0;
    }

    double getRightEncoder() {
        return rightMotorFront.getCurrentPosition();
        // (rightMotorFront.getCurrentPosition() + rightMotorBack.getCurrentPosition()) / 2.0; //TODO fix wiring
    }

    public void Move(double dist) {
        Move(dist, 0, 1);
    }
    public void Move(double dist, double angle) {Move(dist, angle, 1);}
}

