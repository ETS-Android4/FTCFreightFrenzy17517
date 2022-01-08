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
    public static class BrushConfig{
        public static double timeForActivateProtection = 3;
        public static double timeForReverse = timeForActivateProtection + 1;
    }

    @Config
    public static class DuckConfig {
        public static double directionDuck = 1;
    }

    @Config
    public static class MovementConfig {
        public static double dist = 0;
        public static double angle = 0;
        public static double kP_Distance = 0.04;
        public static double kP_Angle = 0.055;
        public static double kI_Distance = 0.001;
        public static double kI_Angle = 0.008;
        public static double kD_Distance = 0.0001;
        public static double kD_Angle = 0.001;
        public static double maxIntegralAngle = 0.25;
        public static double maxIntegralDistance = 0.25;
        public static double minErrorDistance = 5.0;
        public static double minErrorAngle = 2.5;
    }

    @Config
    public static class ManipulatorConfig {
        public static boolean AutoTele = false;
        public static double positionServoUp = 0.38;
        public static double positionServoDown = 0.8;
        public static double positonServoForElevator = 0.4;
    }

    @Config
    public static class TeleOpConfig {
        public static double robotSpeed = 1.0;
    }

    @Config
    public static class Elevator {
        public static double downTargetElevator = 0;
        public static double middleTargetElevator = 650;
        public static double upTargetElevator = 1300;
        public static double bucketServoDelay = 1;
    }
}
