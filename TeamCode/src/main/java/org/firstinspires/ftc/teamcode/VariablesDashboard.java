package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;

public class VariablesDashboard {
    @Config
    public static class MovementConfig {
        public static double kP_Distance = 0.01;
        public static double kP_Angle = 0.015;
        public static double kI_Distance = 0.0001;
        public static double kI_Angle = 0.002;
        public static double kD_Distance = 0.001;
        public static double kD_Angle = 0.0001;
        public static double maxIntegralAngle = 0.25;
        public static double maxIntegralDistance = 0.25;
        public static double minErrorDistance = 5.0;
        public static double minErrorAngle = 5.0;
    }
    @Config
    public static class ManipulatorConfig{
        public static double manipulatorUp = 0.7;
        public static double manipulatorDown = 0.05;
    }

    @Config
    public static class TeleOpConfig {
        public static double robotSpeed = 1.0;
    }
}
