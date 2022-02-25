package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.misc.AllianceColor;
import org.firstinspires.ftc.teamcode.misc.StartingPosition;
import org.firstinspires.ftc.teamcode.robot.Bucket;
import org.firstinspires.ftc.teamcode.robot.Lift;

@Autonomous
public class AutonomRedTeamLeftPos extends BaseDetectionAutonomous {

    Runnable[] upPosition = {

            () -> {
                robot.movement.Move(-60, 25);
                robot.lift.setElevatorTarget(Lift.ElevatorPosition.UP);
            },
            () -> robot.bucket.setBucketPosition(Bucket.BucketPosition.EJECT),
            () -> robot.bucket.setBucketPosition(Bucket.BucketPosition.COLLECT),
            () -> {
                robot.lift.setElevatorTarget(Lift.ElevatorPosition.DOWN);
                robot.movement.Move(-5, 37);
            },
            () -> robot.movement.Move(-15, -90),
            () -> robot.movement.Move(-70, -90, 0.5),
            () -> {
                robot.duck.duckSpin(true);
                robot.movement.Move(-65, -90);
            },
            () -> robot.movement.Move(-30, -90),
            () -> robot.movement.Move(15, -120),
            () -> robot.movement.Move(20, -90),
            () -> robot.movement.Move(110, -90, 1),
            () -> robot.movement.Move(210, -90, 2),};
    Runnable[] middlePosition = {

            () -> {
                robot.movement.Move(-60, 25);                                //-64, -37
                robot.lift.setElevatorTarget(Lift.ElevatorPosition.MIDDLE);
            },
            () -> robot.bucket.setBucketPosition(Bucket.BucketPosition.EJECT),
            () -> robot.bucket.setBucketPosition(Bucket.BucketPosition.COLLECT),
            () -> {
                robot.lift.setElevatorTarget(Lift.ElevatorPosition.DOWN);
                robot.movement.Move(-5, 37);
            },
            () -> robot.movement.Move(-15, -90),
            () -> robot.movement.Move(-70, -90, 0.5),
            () -> {
                robot.duck.duckSpin(true);
                robot.movement.Move(-65, -90);
            },
            () -> robot.movement.Move(-30, -90),
            () -> robot.movement.Move(15, -120),
            () -> robot.movement.Move(20, -90),
            () -> robot.movement.Move(110, -90, 1),
            () -> robot.movement.Move(210, -90, 2),};
    Runnable[] downPosition = {

            () -> {
                robot.movement.Move(-60, 25);
                robot.lift.setElevatorTarget(Lift.ElevatorPosition.DOWN);
            },
            () -> robot.bucket.setBucketPosition(Bucket.BucketPosition.EJECT),
            () -> robot.bucket.setBucketPosition(Bucket.BucketPosition.COLLECT),
            () -> {
                robot.lift.setElevatorTarget(Lift.ElevatorPosition.DOWN);
                robot.movement.Move(-5, 37);
            },
            () -> robot.movement.Move(-15, -90),
            () -> robot.movement.Move(-70, -90, 0.5),
            () -> {
                robot.duck.duckSpin(true);
                robot.movement.Move(-65, -90);
            },
            () -> robot.movement.Move(-30, -90),
            () -> robot.movement.Move(15, -120),
            () -> robot.movement.Move(20, -90),
//            () -> robot.timer.delay(4),

            () -> robot.movement.Move(110, -90, 1),
            () -> robot.movement.Move(210, -90, 2),};

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
        robot.duck.setFieldPosition(AllianceColor.RED, StartingPosition.LEFT);
        super.runOpMode();
    }
}