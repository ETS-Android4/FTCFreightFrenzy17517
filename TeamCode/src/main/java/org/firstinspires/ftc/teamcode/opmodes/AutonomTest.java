package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.teamcode.opmodes.AutonomTest.AutonomTestConfig.x;
import static org.firstinspires.ftc.teamcode.opmodes.AutonomTest.AutonomTestConfig.y;

import static java.lang.Math.toRadians;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.misc.Pose2D;
import org.firstinspires.ftc.teamcode.robot.Bucket;
import org.firstinspires.ftc.teamcode.robot.Lift;

@Autonomous
public class AutonomTest extends BaseAutonomous {

    Runnable[] test = {                         //-64, -37
            () -> {
                robot.odometry.setCurrentPosition(new Pose2D(24.6, 161.16, toRadians(-90)));
         //       robot.globalMovement.Move(-1,103);
                robot.lift.setElevatorTarget(Lift.ElevatorPosition.UP);
            },
            () -> robot.bucket.setBucketPosition(Bucket.BucketPosition.EJECT),
            () -> robot.bucket.setBucketPosition(Bucket.BucketPosition.COLLECT),
            () -> robot.lift.setElevatorTarget(Lift.ElevatorPosition.DOWN),
          //  () -> robot.globalMovement.Move(6, 115),
          //  () -> robot.globalMovement.Move(157, 104),
    };

    @Override
    public void main() {
            execute(test);
    }
    @Config
    public static class AutonomTestConfig {
        public static double y = 0;
        public static double x =0;
     }
}
