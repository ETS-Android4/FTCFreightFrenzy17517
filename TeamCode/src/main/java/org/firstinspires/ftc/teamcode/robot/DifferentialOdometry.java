package org.firstinspires.ftc.teamcode.robot;

import static java.lang.Math.PI;
import static java.lang.Math.abs;
import static java.lang.Math.cos;
import static java.lang.Math.sin;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.misc.Pose2D;

public class DifferentialOdometry implements RobotModule {

    private static final double TRACK_WIDTH_CM = 38.84500;
    private static final double WHEEL_DIAMETER_CM = 10.90532;
    private static final int ENCODER_RESOLUTION = 24;
    private static final double GEARBOX_RATIO = 20.0;
    private static final double ENCODER_TICKS_TO_CM_RATIO =
            (WHEEL_DIAMETER_CM * PI) / (ENCODER_RESOLUTION * GEARBOX_RATIO);
    private static final double CM_TO_ROTATION_DEGREES_RATIO =
            (180.0 / ((TRACK_WIDTH_CM / 2.0) * PI)) * (3600.0 / (3600.0 + (128.868 + 115.076) * 0.5));
    private static Pose2D currentPosition = new Pose2D();
    private final WoENRobot robot;
    private DcMotorEx leftMotorFront = null;
    private DcMotorEx leftMotorBack = null;
    private DcMotorEx rightMotorFront = null;
    private DcMotorEx rightMotorBack = null;
    private double previousLeftEncoder = 0;
    private double previousRightEncoder = 0;
    private double previousHeading = 0;

    public DifferentialOdometry(WoENRobot robot) {
        this.robot = robot;
    }

    double getLeftEncoder() {
        return (leftMotorBack.getCurrentPosition() + leftMotorFront.getCurrentPosition()) / 2.0;
    }

    double getRightEncoder() {
        return rightMotorFront.getCurrentPosition();
        // (rightMotorFront.getCurrentPosition() + rightMotorBack.getCurrentPosition()) / 2.0; //TODO fix wiring
    }

    public double getVelocityKmH(){
        return encoderTicksToCm((leftMotorBack.getVelocity() + leftMotorFront.getVelocity()
                + rightMotorFront.getVelocity() * 2.0 /*+ rightMotorBack.getCurrentPosition() */)/4.0)*(3600.0/(100.0*1000.0));
    }

    public Pose2D getCurrentPosition() {
        return currentPosition;
    }

    public void setCurrentPosition(Pose2D currentPosition) {
        this.currentPosition = currentPosition;
    }

    @Override
    public void initialize() {
        assignHardware();
        setDirections();
    }

    @Override
    public void update() {
        double currentLeftEncoder = getLeftEncoder();
        double currentRightEncoder = getRightEncoder();
        double currentHeading = Math.toRadians(getGyroHeading());
        //double currentHeading = Math.toRadians(encoderTicksToRotationDegrees(currentLeftEncoder - currentRightEncoder) * 0.5));
        double deltaLeftEncoder = currentLeftEncoder - previousLeftEncoder;
        double deltaRightEncoder = currentRightEncoder - previousRightEncoder;
        double deltaHeading = angleWrapHalf(currentHeading - previousHeading);

        double deltaHeadingEncoder =
                Math.toRadians(encoderTicksToRotationDegrees((deltaLeftEncoder - deltaRightEncoder) * 0.5));

        Pose2D deltaPosition =
                new Pose2D(encoderTicksToCm(deltaLeftEncoder + deltaRightEncoder) * 0.5, 0, deltaHeading);
        if (deltaHeadingEncoder != 0.0) {   //if deltaAngle = 0 radius of the arc is = Inf which causes model degeneracy
            double arcAngle = deltaHeadingEncoder * 2.0;
            double arcRadius = abs(deltaPosition.x) / arcAngle;
            deltaPosition = new Pose2D(arcRadius * (1 - cos(arcAngle)), arcRadius * sin(arcAngle), deltaHeading)
                    .rotatedCW(deltaPosition.aCot());
        }
        currentPosition = currentPosition.plus(deltaPosition.rotatedCW(currentPosition.heading));

        previousLeftEncoder = currentLeftEncoder;
        previousRightEncoder = currentRightEncoder;
        previousHeading = currentHeading;
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
        rightMotorBack.setDirection(DcMotorEx.Direction.FORWARD);
        leftMotorFront.setDirection(DcMotorEx.Direction.REVERSE);
        leftMotorBack.setDirection(DcMotorEx.Direction.REVERSE);
    }

    private double angleWrapHalf(double angle) {
        while (angle > PI / 2) angle -= PI;
        while (angle < -PI / 2) angle += PI;
        return angle;
    }
}
