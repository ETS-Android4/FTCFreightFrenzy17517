package org.firstinspires.ftc.teamcode.opmodes;

import static java.lang.Math.toRadians;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.misc.AllianceColor;
import org.firstinspires.ftc.teamcode.misc.Pose2D;
import org.firstinspires.ftc.teamcode.misc.StartingPosition;
import org.firstinspires.ftc.teamcode.robot.Bucket;
import org.firstinspires.ftc.teamcode.robot.Lift;

@Autonomous
public class AutonomBlueTeamRIghtPos extends BaseDetectionAutonomous {

    Runnable[] downPosition = {
            () -> {
                robot.movement.Move(-58, -37);                                //-64, -37
                robot.lift.setElevatorTarget(Lift.ElevatorPosition.DOWN);
            },
            () -> robot.bucket.setBucketPosition(Bucket.BucketPosition.EJECT),
            () -> robot.bucket.setBucketPosition(Bucket.BucketPosition.COLLECT),
            () -> robot.lift.setElevatorTarget(Lift.ElevatorPosition.DOWN),
            () -> robot.movement.Move(-25.72, -37),
            () -> robot.movement.Move(-25.72, 90),
            () -> robot.movement.Move(-94, 90),
            () -> robot.movement.Move(-94, 180),
            () -> robot.movement.Move(-101, 180),
            () -> {
                robot.duck.duckSpin(true);
                robot.movement.Move(-101.11, 180);
            },
            () -> robot.movement.Move(-77.16, 180),
            () -> robot.movement.Move(-77.16, 90),
            () -> robot.movement.Move(216.05, 90, 1.4),
            () -> robot.movement.Move(216.05, 0),
            () -> robot.movement.Move(220, 0, 1.7),


    };
    Runnable[] middlePosition = {
            () -> {
                robot.movement.Move(-58, -37);                                //-64, -37
                robot.lift.setElevatorTarget(Lift.ElevatorPosition.MIDDLE);
            },
            () -> robot.bucket.setBucketPosition(Bucket.BucketPosition.EJECT),
            () -> robot.bucket.setBucketPosition(Bucket.BucketPosition.COLLECT),
            () -> robot.lift.setElevatorTarget(Lift.ElevatorPosition.DOWN),
            () -> robot.movement.Move(-25.72, -37),
            () -> robot.movement.Move(-25.72, 90),
            () -> robot.movement.Move(-94, 90),
            () -> robot.movement.Move(-94, 180),
            () -> robot.movement.Move(-101, 180),
            () -> {
                robot.duck.duckSpin(true);
                robot.movement.Move(-101.11, 180);
            },
            () -> robot.movement.Move(-77.16, 180),
            () -> robot.movement.Move(-77.16, 90),
            () -> robot.movement.Move(216.05, 90, 1.4),
            () -> robot.movement.Move(216.05, 0),
            () -> robot.movement.Move(220, 0, 1.7),

    };
    Runnable[] upPosition = {

            () -> {
                robot.movement.Move(-58, -37);                                //-64, -37
                robot.lift.setElevatorTarget(Lift.ElevatorPosition.UP);
            },
            () -> robot.bucket.setBucketPosition(Bucket.BucketPosition.EJECT),
            () -> robot.bucket.setBucketPosition(Bucket.BucketPosition.COLLECT),
            () -> robot.lift.setElevatorTarget(Lift.ElevatorPosition.DOWN),
            () -> robot.movement.Move(-25.72, -37),
            () -> robot.movement.Move(-25.72, 90),
            () -> robot.movement.Move(-94, 90),
            () -> robot.movement.Move(-94, 180),
            () -> robot.movement.Move(-101, 180),
            () -> {
                robot.duck.duckSpin(true);
                robot.movement.Move(-101.11, 180);
            },
            () -> robot.movement.Move(-77.16, 180),
            () -> robot.movement.Move(-77.16, 90),
            () -> robot.movement.Move(216.05, 90, 1.4),
            () -> robot.movement.Move(216.05, 0),
            () -> robot.movement.Move(220, 0, 1.7),


    };

    @Override
    protected Runnable[] upPosition() {
        return upPosition;
    }

    @Override
    protected Runnable[] middlePosition() {
        return middlePosition;
    }

    @Override
    protected Runnable[] downPosition() {
        return downPosition;
    }

    @Override
    public void runOpMode() {
        robot.duck.setFieldPosition(AllianceColor.BLUE, StartingPosition.RIGHT);
        robot.odometry.setCurrentPosition(new Pose2D(-79.036, 154.127, toRadians(-90)));
        super.runOpMode();
    }
}