package org.firstinspires.ftc.teamcode.opmodes;

import static java.lang.Math.toRadians;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.sun.tools.javac.util.ArrayUtils;

import org.firstinspires.ftc.teamcode.misc.AllianceColor;
import org.firstinspires.ftc.teamcode.misc.Pose2D;
import org.firstinspires.ftc.teamcode.misc.StartingPosition;
import org.firstinspires.ftc.teamcode.robot.Bucket;
import org.firstinspires.ftc.teamcode.robot.Lift;

@Autonomous
public class AutonomRedTeamLeftPos extends BaseDetectionAutonomous {

    Runnable[] upPosition = {

            () -> {
                robot.movement.Move(-63, 40, 0.8);
                robot.lift.setElevatorTarget(Lift.ElevatorPosition.UP);
            },
            () -> {robot.bucket.setBucketPosition(Bucket.BucketPosition.EJECT);
            },
            () -> {robot.bucket.setBucketPosition(Bucket.BucketPosition.COLLECT);
            },
            () -> {
                robot.lift.setElevatorTarget(Lift.ElevatorPosition.DOWN);
                robot.movement.Move(0, 32);
            },
            () -> robot.movement.Move(0, -90),
            () -> robot.movement.Move(-40, -90, 0.4),
            () -> {
                robot.duck.duckSpin(true);
                robot.movement.Move(-40, -90);
            },
            () -> robot.movement.Move(-25, -90),
            () -> robot.movement.Move(25, -120),
            () -> robot.movement.Move(20.58, -90),
            () -> robot.movement.Move(90, -90, 1),
            () -> robot.movement.Move(250, -90, 2),

    };
    Runnable[] middlePosition = {

            () -> {
                robot.movement.Move(-61.73, 28, 0.8);
                robot.lift.setElevatorTarget(Lift.ElevatorPosition.MIDDLE);
            },
            () -> {robot.bucket.setBucketPosition(Bucket.BucketPosition.EJECT);
            },
            () -> {robot.bucket.setBucketPosition(Bucket.BucketPosition.COLLECT);
            },
            () -> {
                robot.lift.setElevatorTarget(Lift.ElevatorPosition.DOWN);
                robot.movement.Move(-5.14, 28);
            },
            () -> robot.movement.Move(-15.43, -90, 0.3),
            () -> robot.movement.Move(-40, -90, 0.2),
            () -> {
                robot.duck.duckSpin(true);
                robot.movement.Move(-40, -90);
            },
     /*
            () -> robot.movement.Move(-30.86, -90),
            () -> robot.movement.Move(15, -120),
            () -> robot.movement.Move(20.58, -90),
            () -> robot.movement.Move(113.17, -90, 1),
            () -> robot.movement.Move(216.05, -90, 2),
    */
    };
    Runnable[] downPosition = {
            () -> {
                robot.movement.Move(-61.73, 28, 0.8);
                robot.lift.setElevatorTarget(Lift.ElevatorPosition.DOWN);
            },
            () -> {robot.bucket.setBucketPosition(Bucket.BucketPosition.EJECT);
            },
            () -> {robot.bucket.setBucketPosition(Bucket.BucketPosition.COLLECT);
            },
            () -> {
                robot.lift.setElevatorTarget(Lift.ElevatorPosition.DOWN);
                robot.movement.Move(-5.14, 28);
            },
            () -> robot.movement.Move(-15.43, -90, 0.3),
            () -> robot.movement.Move(-40, -90, 0.2),
            () -> {
                robot.duck.duckSpin(true);
                robot.movement.Move(-40, -90);
            },
            /*
            () -> robot.movement.Move(-30.86, -90),
            () -> robot.movement.Move(15, -120),
            () -> robot.movement.Move(20.58, -90),
            () -> robot.movement.Move(113.17, -90, 1),
            () -> robot.movement.Move(216.05, -90, 2),*/};
    @Override
    protected Runnable[] upPosition() {
        return downPosition;
    }

    @Override
    protected Runnable[] middlePosition() {
        return middlePosition;
    }

    @Override
    protected Runnable[] downPosition() {
        return upPosition;
    }

    @Override
    public void runOpMode() {
        robot.duck.setFieldPosition(AllianceColor.RED, StartingPosition.LEFT);
        robot.odometry.setCurrentPosition(new Pose2D(-79.036, -154.127, toRadians(-90)));
        super.runOpMode();
    }
}