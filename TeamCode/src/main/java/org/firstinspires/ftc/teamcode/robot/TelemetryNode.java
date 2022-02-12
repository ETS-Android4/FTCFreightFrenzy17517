package org.firstinspires.ftc.teamcode.robot;

import static org.firstinspires.ftc.teamcode.robot.TelemetryNode.TelemetryConfig.msTransmissionInterval;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

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
        if (transmissionTimer.milliseconds() < msTransmissionInterval) currentTelemetry.clear();
        else {
            transmissionTimer.reset();
            currentTelemetry.update();
        }
    }

    public enum TelemetryType {
        DRIVER_STATION, DASHBOARD, DUAL
    }

    @Config
    public static class TelemetryConfig {
        public static TelemetryNode.TelemetryType telemetryType =
                TelemetryNode.TelemetryType.DRIVER_STATION;
        public static int msTransmissionInterval = 250;
    }
}
