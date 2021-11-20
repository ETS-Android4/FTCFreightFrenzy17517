package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;

public class VariablesDashboard {

    @Config
    public static class MovementConfig {
        public static double dist=0;
        public static double angle=0;
        public static double kP_Distance = 0.04;
        public static double kP_Angle = 0.01;
        public static double kI_Distance = 0.001;
        public static double kI_Angle = 0.005;
        public static double kD_Distance = 0.0001;
        public static double kD_Angle = 0.001;
        public static double maxIntegralAngle = 0.25;
        public static double maxIntegralDistance = 0.25;
        public static double minErrorDistance = 5.0;
        public static double minErrorAngle = 5.0;
    }
    @Config
    public static class ManipulatorConfig{
        public static double positionServoUp = 0.38;
        public static double positionServoDown = 0.78;
        public static double positonServoForElevator = 0.48;
    }

    @Config
    public static class TeleOpConfig {
        public static double robotSpeed = 1.0;
    }
    @Config
    public static class Elevator{
        public static double DownTargetElevator = 0;
        public static double MiddleTargetElevator = 1600;
        public static double UpTargetElevator = 3900;
    }
}
