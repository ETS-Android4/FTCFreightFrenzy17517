package org.firstinspires.ftc.teamcode.robot;

import static org.firstinspires.ftc.teamcode.robot.TelemetryNode.TelemetryConfig.msTransmissionInterval;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class TelemetryNode implements RobotModule {

    private final ElapsedTime transmissionTimer = new ElapsedTime();
    private WoENRobot robot = null;
    private FtcDashboard dashboard = null;
    private Telemetry dashboardTelemetry = null;
    private Telemetry opModeTelemetry = null;
    private Telemetry multipleTelemetry = null;
    private Telemetry currentTelemetry = null;


    public TelemetryNode(WoENRobot robot) {
        this.robot = robot;
    }

    private Telemetry choseTelemetry() {
        switch (TelemetryConfig.telemetryType) {
            case DASHBOARD:
                return dashboardTelemetry;
            case DUAL:
                return multipleTelemetry;
            case DRIVER_STATION:
            default:
                return opModeTelemetry;
        }
    }

    public Telemetry getTelemetry() {
        return currentTelemetry;
    }

    @Override
    public void initialize() {
        dashboard = FtcDashboard.getInstance();
        dashboardTelemetry = dashboard.getTelemetry();
        opModeTelemetry = robot.getLinearOpMode().telemetry;
        multipleTelemetry = new MultipleTelemetry(dashboardTelemetry, opModeTelemetry);
        multipleTelemetry.setMsTransmissionInterval(msTransmissionInterval);
        currentTelemetry = choseTelemetry();
        currentTelemetry.clearAll();
        transmissionTimer.reset();
    }

    @Override
    public void update() {
        if (transmissionTimer.milliseconds() > msTransmissionInterval) {
            currentTelemetry.addData("Mode", TelemetryNode.TelemetryModuleConfig.telemetryModuleValue);
            switch (TelemetryNode.TelemetryModuleConfig.telemetryModuleValue) {
                case ACCUMULATOR:
                    currentTelemetry.addData("Voltage", robot.accumulator.getBatteryVoltage());
                    currentTelemetry.addData("Coefficient", robot.accumulator.getkVoltage());
                    break;
                case BRUSH:
                    currentTelemetry.addData("Enabled", robot.brush.getEnableIntake());
                    currentTelemetry.addData("Motor current", robot.brush.getBrushMotorCurrent());
                    currentTelemetry.addData("Protection enabled", robot.brush.protectionBrushMotor());
                    break;
                case BUCKET:
                    currentTelemetry.addData("Bucket position", robot.bucket.getBucketPosition());
                    currentTelemetry.addData("Freight detected", robot.bucket.isFreightDetected());
                    currentTelemetry.addData("Servo timer", robot.bucket.servoTimer);
                    break;
                case ENCODERS:
                    currentTelemetry.addData("Left encoder", robot.movement.getLeftEncoder());
                    currentTelemetry.addData("Right encoder", robot.movement.getRightEncoder());
                    break;
                case GYRO:
                    currentTelemetry.addData("Heading", -robot.gyro.getOrientation().firstAngle);
                    currentTelemetry.addData("Roll", robot.gyro.getOrientation().secondAngle);
                    currentTelemetry.addData("Pitch", robot.gyro.getOrientation().thirdAngle);
                    break;
                case LIFT:
                    currentTelemetry.addData("ElevatorPosition", robot.lift.getElevatorPosition());
                    currentTelemetry.addData("LiftEncoderPosition", robot.lift.getLiftEncoderPosition());
                    break;
                case MOVEMENT:
                    robot.movement.telemetryForMovement(currentTelemetry);
                    break;
                case LEDSTRIP:
                    currentTelemetry.addData("LED Current", robot.ledStrip.getLEDCurrent());
                    break;
                case DUCK:
                    currentTelemetry.addData("Done spinning",robot.duck.actionIsCompleted());
                    break;
                case UNEXPECTED:
                    currentTelemetry.addData("chto-to ne ochevidnoe", 0);
                    currentTelemetry.addData("Odometry", robot.odometry.getCurrentPosition());
                    break;
                case OPENCV:
                    currentTelemetry.addData("Camera", robot.arucoDetect.forceGetPosition());
            }
            transmissionTimer.reset();
            currentTelemetry.update();
        }
    }

    public enum TelemetryType {
        DRIVER_STATION, DASHBOARD, DUAL
    }

    public enum TelemetryModuleValue {
        ACCUMULATOR, BRUSH, BUCKET, ENCODERS, OPENCV, GYRO, LIFT, MOVEMENT, LEDSTRIP, DUCK, UNEXPECTED
    }

    @Config
    public static class TelemetryConfig {
        public static TelemetryNode.TelemetryType telemetryType = TelemetryType.DASHBOARD;
        public static int msTransmissionInterval = 250;
    }

    @Config
    public static class TelemetryModuleConfig {
        public static TelemetryModuleValue telemetryModuleValue = TelemetryModuleValue.ACCUMULATOR;
    }
}
