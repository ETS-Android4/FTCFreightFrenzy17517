package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.misc.AllianceColor;
import org.firstinspires.ftc.teamcode.misc.StartingPosition;
import org.firstinspires.ftc.teamcode.robot.Bucket;
import org.firstinspires.ftc.teamcode.robot.Lift;

@Autonomous
public class AutonomBlueTeamLeftPos extends BaseDetectionAutonomous {

    Runnable[] downPosition = {

            () -> {
                robot.movement.Move(-60, 30);
                robot.lift.setElevatorTarget(Lift.ElevatorPosition.DOWN);
            },
            () -> robot.bucket.setBucketPosition(Bucket.BucketPosition.EJECT),
            () -> robot.bucket.setBucketPosition(Bucket.BucketPosition.COLLECT),
            () -> {
                robot.lift.setElevatorTarget(Lift.ElevatorPosition.DOWN);
            },
            () -> {
                robot.movement.Move(-32, 30);
            },
            () -> {
                robot.movement.Move(-32, 90);
            },
            () -> {
                robot.movement.Move(110, 90, 1.6);
            },

    };
    Runnable[] middlePosition = {

            () -> {
                robot.movement.Move(-60, 30);
                robot.lift.setElevatorTarget(Lift.ElevatorPosition.MIDDLE);
            },
            () -> robot.bucket.setBucketPosition(Bucket.BucketPosition.EJECT),
            () -> robot.bucket.setBucketPosition(Bucket.BucketPosition.COLLECT),
            () -> {
                robot.lift.setElevatorTarget(Lift.ElevatorPosition.DOWN);
            },
            () -> {
                robot.movement.Move(-32, 30);
            },
            () -> {
                robot.movement.Move(-32, 90);
            },
            () -> {
                robot.movement.Move(110, 90, 1.6);
            },};
    Runnable[] upPosition = {

            () -> {
                robot.movement.Move(-60, 30, 1.5);
                robot.lift.setElevatorTarget(Lift.ElevatorPosition.UP);
            },
            () -> robot.bucket.setBucketPosition(Bucket.BucketPosition.EJECT),
            () -> robot.bucket.setBucketPosition(Bucket.BucketPosition.COLLECT),
            () -> {
                robot.lift.setElevatorTarget(Lift.ElevatorPosition.DOWN);
            },
            () -> {
                robot.movement.Move(-32, 30);
            },
            () -> {
                robot.movement.Move(-32, 90);
            },
            () -> {
                robot.movement.Move(110, 90, 1.6);
            },};

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
        robot.duck.setFieldPosition(AllianceColor.BLUE, StartingPosition.LEFT);
        super.runOpMode();
    }
}