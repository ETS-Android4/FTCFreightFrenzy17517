package org.firstinspires.ftc.teamcode.robot;

import static org.firstinspires.ftc.teamcode.robot.TelemetryNode.TelemetryConfig.msTransmissionInterval;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class TelemetryNode implements RobotModule {

    private WoENRobot robot = null;
    private FtcDashboard dashboard = null;
    private Telemetry dashboardTelemetry = null;
    private Telemetry opModeTelemetry = null;
    private Telemetry multipleTelemetry = null;
    private Telemetry currentTelemetry = null;

    private ElapsedTime transmissionTimer = new ElapsedTime();


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
        if (transmissionTimer.milliseconds() > msTransmissionInterval)
        {
            switch(TelemetryNode.TelemetryModuleConfig.telemetryModuleValue){
                case ACCUMULATOR:
                    currentTelemetry.addData("Accumulator", robot.accumulator.getkVoltage());
                    currentTelemetry.addData("Accumulator", robot.accumulator.timedVoltageSensor.getValue());
                    break;
                case BRUSH:
                    currentTelemetry.addData("Protection(On or Off)", robot.brush.protectionBrushMotor());
                    currentTelemetry.addData("EnableIntake", robot.brush.getEnableIntake());
                    currentTelemetry.addData("AMPS brush motor", robot.brush.brushMotor.getCurrent(CurrentUnit.AMPS));
                    break;
                case BUCKET:
                    currentTelemetry.addData("BucketPosition", robot.bucket.getBucketPosition());
                    currentTelemetry.addData("ServoTimer", robot.bucket.servoTimer);
                    break;
                case ENCODERS:
                    currentTelemetry.addData("RightEncoder", robot.movement.getRightEncoder());
                    currentTelemetry.addData("LeftEncoder", robot.movement.getLeftEncoder());
                    break;
                case GYRO:
                    currentTelemetry.addData("Orientation",robot.gyro.getOrientation());
                    break;
                case LIFT:
                    currentTelemetry.addData("ElevatorPosition",robot.lift.getElevatorPosition());
                    currentTelemetry.addData("LiftEncoderPosition",robot.lift.getLiftEncoderPosition());
                    break;
                case MOVEMENT:
                    robot.movement.telemetryForMovement();
                    break;
                case UNEXPECTED:
                    currentTelemetry.addData("chto-to ne ochevidnoe", 0);
                    break;
                case OPENCV:
                    currentTelemetry.addData("Camera", robot.arucoDetect.forceGetPosition());
            }
            currentTelemetry.addData("Mode", TelemetryNode.TelemetryModuleConfig.telemetryModuleValue);
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
        public static TelemetryNode.TelemetryType telemetryType =
                TelemetryType.DUAL;
        public static int msTransmissionInterval = 250;
    }

    @Config
    public static class TelemetryModuleConfig{
        public static TelemetryModuleValue telemetryModuleValue = TelemetryModuleValue.ACCUMULATOR;
    }
}
