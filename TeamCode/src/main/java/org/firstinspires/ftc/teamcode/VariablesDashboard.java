package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;

public abstract class VariablesDashboard {
    @Config
    public static class TelemetryConfig {
        public static TelemetryType telemetryType = TelemetryType.DRIVER_STATION;

        public enum TelemetryType {
            DRIVER_STATION,
            DASHBOARD,
            DUAL
        }
    }

    @Config
    public static class ManipulatorConfig {
        public static boolean AutoTele = false;
    }

    @Config
    public static class TeleOpConfig {
        public static double robotSpeed = 1.0;
    }

}
