package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.misc.AllianceColor;
import org.firstinspires.ftc.teamcode.misc.StartingPosition;
import org.firstinspires.ftc.teamcode.robot.Bucket;
import org.firstinspires.ftc.teamcode.robot.Lift;

@Autonomous
public class AutonomRedTeamRightPos extends BaseDetectionAutonomous {

    Runnable[] upPosition = {
            () -> robot.timer.delay(11),
            () -> {
                robot.movement.Move(-59.57, -30);
                robot.lift.setElevatorTarget(Lift.ElevatorPosition.UP);
                robot.timer.delay(1);
            },
            () -> robot.bucket.setBucketPosition(Bucket.BucketPosition.EJECT),
            () -> robot.bucket.setBucketPosition(Bucket.BucketPosition.COLLECT),
            () -> robot.lift.setElevatorTarget(Lift.ElevatorPosition.DOWN),
            () -> robot.movement.Move(-54.53, -30, 5),
            () -> robot.movement.Move(-46.30, 0, 4),
            () -> robot.movement.Move(-46.30, -90, 4),
            () -> robot.timer.delay(1),
            () -> robot.movement.Move(100, -90, 5),};
    Runnable[] middlePosition = {

            () -> robot.timer.delay(11),
            () -> {
                robot.movement.Move(-59.57, -30);
                robot.lift.setElevatorTarget(Lift.ElevatorPosition.MIDDLE);
            },
            () -> robot.bucket.setBucketPosition(Bucket.BucketPosition.EJECT),
            () -> robot.bucket.setBucketPosition(Bucket.BucketPosition.COLLECT),
            () -> robot.lift.setElevatorTarget(Lift.ElevatorPosition.DOWN),
            () -> robot.movement.Move(-54.53, -30, 5),
            () -> robot.movement.Move(-46.30, 0, 4),
            () -> robot.movement.Move(-46.30, -90, 4),
            () -> robot.timer.delay(1),
            () -> robot.movement.Move(100, -90, 5),};

    Runnable[] downPosition = {

            () -> robot.timer.delay(11),
            () -> {
                robot.movement.Move(-59.57, -30);
                robot.lift.setElevatorTarget(Lift.ElevatorPosition.DOWN);
            },
            () -> robot.bucket.setBucketPosition(Bucket.BucketPosition.EJECT),
            () -> robot.bucket.setBucketPosition(Bucket.BucketPosition.COLLECT),
            () -> robot.lift.setElevatorTarget(Lift.ElevatorPosition.DOWN),
            () -> robot.movement.Move(-54.53, -30, 5),
            () -> robot.movement.Move(-46.30, 0, 4),
            () -> robot.movement.Move(-46.30, -90, 4),
            () -> robot.timer.delay(1),
            () -> robot.movement.Move(100, -90, 5),};
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
        robot.duck.setFieldPosition(AllianceColor.RED, StartingPosition.RIGHT);
        super.runOpMode();
    }
}