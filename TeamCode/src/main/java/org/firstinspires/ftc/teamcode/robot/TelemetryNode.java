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
            currentTelemetry.addLine().addData("Mode", TelemetryNode.TelemetryModuleConfig.telemetryModuleValue);
            switch (TelemetryNode.TelemetryModuleConfig.telemetryModuleValue) {
                case ACCUMULATOR:
                    currentTelemetry.addLine().addData("Voltage", ".3f", robot.accumulator.getBatteryVoltage())
                            .addData("Coefficient", ".3f", robot.accumulator.getkVoltage());
                    break;
                case BRUSH:
                    currentTelemetry.addLine().addData("Enabled", robot.brush.getEnableIntake())
                            .addData("Motor current", ".2f", robot.brush.getBrushMotorCurrent())
                            .addData("Protection enabled", robot.brush.protectionBrushMotor());
                    break;
                case BUCKET:
                    currentTelemetry.addLine().addData("Bucket position", robot.bucket.getBucketPosition())
                            .addData("Freight detected", robot.bucket.isFreightDetected())
                            .addData("Servo timer", ".1f", robot.bucket.servoTimer);
                    break;
                case ENCODERS:
                    currentTelemetry.addLine().addData("Left encoder", ".0f", robot.movement.getLeftEncoder())
                            .addData("Right encoder", ".0f", robot.movement.getRightEncoder());
                    break;
                case GYRO:
                    currentTelemetry.addLine().addData("Heading", ".2f", -robot.gyro.getOrientation().firstAngle)
                            .addData("Roll", ".2f", robot.gyro.getOrientation().secondAngle)
                            .addData("Pitch", ".2f", robot.gyro.getOrientation().thirdAngle);
                    break;
                case LIFT:
                    currentTelemetry.addLine().addData("ElevatorPosition", robot.lift.getElevatorPosition())
                            .addData("LiftEncoderPosition", robot.lift.getLiftEncoderPosition());
                    break;
                case MOVEMENT:
                    robot.movement.telemetryForMovement(currentTelemetry);
                    break;
                case UNEXPECTED:
                    currentTelemetry.addLine().addData("chto-to ne ochevidnoe", 0);
                    break;
                case OPENCV:
                    currentTelemetry.addLine().addData("Camera", robot.arucoDetect.forceGetPosition());
            }
            transmissionTimer.reset();
            currentTelemetry.update();
        }
    }

    public enum TelemetryType {
        DRIVER_STATION, DASHBOARD, DUAL
    }

    public enum TelemetryModuleValue {
        ACCUMULATOR, BRUSH, BUCKET, ENCODERS, OPENCV, GYRO, LIFT, MOVEMENT, UNEXPECTED
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
