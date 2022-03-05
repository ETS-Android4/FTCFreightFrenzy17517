package org.firstinspires.ftc.teamcode.robot;

import static org.firstinspires.ftc.teamcode.robot.TelemetryNode.TelemetryConfig.msTransmissionInterval;

import static java.lang.Math.abs;
import static java.lang.Math.cos;
import static java.lang.Math.nextDown;
import static java.lang.Math.sin;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.misc.Pose2D;

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
                case SPEEDOMETER:
                    double velocity = robot.odometry.getVelocityKmH();
                    double current = robot.movement.getCurrent();
                    String velocityString = "";
                    String currentString = "";
                    for(int i = 0; i < Math.round(abs(velocity) / 0.1); i++)
                        velocityString += "/";
                    for(int i = 0; i < Math.round(current / 0.33); i++)
                        currentString += "/";
                    currentTelemetry.addData("Velocity (km/h)", robot.odometry.getVelocityKmH());
                    currentTelemetry.addLine(velocityString);
                    currentTelemetry.addData("Current (A)", robot.movement.getCurrent());
                    currentTelemetry.addLine(currentString);
                    break;
                case ACCUMULATOR:
                    currentTelemetry.addData("Voltage", robot.battery.getBatteryVoltage());
                    currentTelemetry.addData("Coefficient", robot.battery.getkVoltage());
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
                    currentTelemetry.addData("LiftEncoderPosition", robot.lift.getOffsetEncoderPosition());
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
                case ODOMETRY:
                    createDashboardRectangle(robot.odometry.getCurrentPosition(), "black");
                    break;
                case UNEXPECTED:
                    currentTelemetry.addData("chto-to ne ochevidnoe", 0);
                    break;
                case OPENCV:
                    currentTelemetry.addData("Camera", robot.arucoDetect.forceGetPosition());
            }
            transmissionTimer.reset();
            currentTelemetry.update();
        }
    }

    private void createDashboardRectangle(Pose2D position, String color) {
        double by = position.y / 2.54;
        double bx = position.x / 2.54;
        double l = 45.0 / (2.54 * 2.0);
        double l2 = l * 435.55 / 444;
        double[] bxPoints = {l, -l, -l, l};
        double[] byPoints = {l2, l2, -l2, -l2};
        rotatePoints(bxPoints, byPoints, -position.heading);
        for (int i = 0; i <=3; i++) {
            bxPoints[i] += bx;
            byPoints[i] += by;
        }
        TelemetryPacket dashboardPacket = new TelemetryPacket();
        dashboardPacket.put("Odometry", robot.odometry.getCurrentPosition());
        dashboardPacket.fieldOverlay().setStroke("#00EEFF").setStrokeWidth(2) //cyan
                .strokeLine(bx, by, (bxPoints[0] + bxPoints[3]) / 2, (byPoints[0] + byPoints[3]) / 2);
        dashboardPacket.fieldOverlay().setStroke("#FFA700").setStrokeWidth(1) //orange
                .strokeLine(bx, by, (bxPoints[0] + bxPoints[3]) / 2, (byPoints[0] + byPoints[3]) / 2);
        dashboardPacket.fieldOverlay().setStroke(color).setStrokeWidth(1).strokePolygon(bxPoints, byPoints);
        dashboard.sendTelemetryPacket(dashboardPacket);
    }

    private void rotatePoints(double[] xPoints, double[] yPoints, double angle) {
        for (int i = 0; i <= 3; i++) {
            double x = xPoints[i];
            double y = yPoints[i];
            xPoints[i] = x * cos(angle) - y * sin(angle);
            yPoints[i] = x * sin(angle) + y * cos(angle);
        }
    }

    public enum TelemetryType {
        DRIVER_STATION, DASHBOARD, DUAL
    }

    public enum TelemetryModuleValue {
        SPEEDOMETER, ACCUMULATOR, BRUSH, BUCKET, ENCODERS, OPENCV, GYRO, LIFT, MOVEMENT, LEDSTRIP, DUCK, ODOMETRY, UNEXPECTED
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
