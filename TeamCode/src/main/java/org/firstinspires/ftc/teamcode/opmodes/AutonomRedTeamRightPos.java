package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.misc.AllianceColor;
import org.firstinspires.ftc.teamcode.misc.StartingPosition;
import org.firstinspires.ftc.teamcode.robot.Bucket;
import org.firstinspires.ftc.teamcode.robot.Lift;

@Autonomous
public class AutonomRedTeamRightPos extends BaseDetectionAutonomous {

    Runnable[] upPosition = {

            () -> {
                robot.movement.Move(-59.57, -30);
                robot.lift.setElevatorTarget(Lift.ElevatorPosition.UP);
                robot.timer.delay(1);
            },
            () -> robot.bucket.setBucketPosition(Bucket.BucketPosition.EJECT),
            () -> robot.bucket.setBucketPosition(Bucket.BucketPosition.COLLECT),
            () -> robot.movement.Move(-54.53, -30, 5),
            () -> robot.movement.Move(-46.30, 0, 4),
            () -> robot.movement.Move(-46.30, 90, 4),
            () -> robot.movement.Move(-185.18, 90, 3),};

    Runnable[] middlePosition = {

            () -> {
                robot.movement.Move(-59.57, -30);
                robot.lift.setElevatorTarget(Lift.ElevatorPosition.MIDDLE);
            },
            () -> robot.bucket.setBucketPosition(Bucket.BucketPosition.EJECT),
            () -> robot.bucket.setBucketPosition(Bucket.BucketPosition.COLLECT),
            () -> robot.lift.setElevatorTarget(Lift.ElevatorPosition.DOWN),
            () -> robot.movement.Move(-54.53, -30, 5),
            () -> robot.movement.Move(-46.30, 0, 4),
            () -> robot.movement.Move(-46.30, 90, 4),
            () -> robot.movement.Move(-185.18, 90, 3),};

    Runnable[] downPosition = {
            () -> {
                robot.movement.Move(-59.57, -30);
                robot.lift.setElevatorTarget(Lift.ElevatorPosition.DOWN);
            },
            () -> robot.bucket.setBucketPosition(Bucket.BucketPosition.EJECT),
            () -> robot.bucket.setBucketPosition(Bucket.BucketPosition.COLLECT),
            () -> robot.lift.setElevatorTarget(Lift.ElevatorPosition.DOWN),
            () -> robot.movement.Move(-54.53, -30, 5),
            () -> robot.movement.Move(-46.30, 0, 4),
            () -> robot.movement.Move(-46.30, 90, 4),
            () -> robot.movement.Move(-185.18, 90, 3),};

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
        robot.duck.setFieldPosition(AllianceColor.RED, StartingPosition.RIGHT);
        super.runOpMode();
    }
}