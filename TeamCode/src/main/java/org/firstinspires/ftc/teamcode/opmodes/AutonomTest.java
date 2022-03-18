package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.teamcode.opmodes.AutonomTest.AutonomTestConfig.*;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.misc.Pose2D;

@Autonomous
public class AutonomTest extends BaseAutonomous {

    Runnable[] test = {                         //-64, -37
            () -> robot.odometry.setCurrentPosition(new Pose2D()),
            () -> robot.globalMovement.Move(x1, y1,a1),
            () -> robot.globalMovement.Move(x2, y2, a2)
    };

    @Override
    public void main() {
        execute(test);
    }

    @Config
    public static class AutonomTestConfig {
        public static double y1 = 30;
        public static double x1 = -40;
        public static double y2 = 30;
        public static double x2 = 30;
        public static double a1 = -30;
        public static double a2 = 60;
    }
}
