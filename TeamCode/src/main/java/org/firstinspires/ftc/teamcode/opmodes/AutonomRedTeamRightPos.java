package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.misc.PositionOnField;
import org.firstinspires.ftc.teamcode.misc.PositionToSearch;
import org.firstinspires.ftc.teamcode.robot.Bucket;
import org.firstinspires.ftc.teamcode.robot.Lift;

@Autonomous
public class AutonomRedTeamRightPos extends BaseDetectionAutonomous {

    Runnable[] upPosition = {

            () -> { robot.movement.Move(-59, -30);
                robot.lift.setElevatorTarget(Lift.ElevatorPosition.DOWN);
                robot.timer.delay(1);},
            () -> robot.bucket.setBucketPosition(Bucket.BucketPosition.EJECT),
            () -> robot.bucket.setBucketPosition(Bucket.BucketPosition.COLLECT),
            () -> {robot.movement.Move(-40, 0);},
            () -> {robot.movement.Move(-40,90);},
            () -> {robot.movement.Move(-220,90);},

    };
    Runnable[] middlePosition = {

            () -> { robot.movement.Move(-59, -30);
                robot.lift.setElevatorTarget(Lift.ElevatorPosition.MIDDLE);},
            () -> robot.bucket.setBucketPosition(Bucket.BucketPosition.EJECT),
            () -> robot.bucket.setBucketPosition(Bucket.BucketPosition.COLLECT),
            () -> {robot.lift.setElevatorTarget(Lift.ElevatorPosition.DOWN); },
            () -> {robot.movement.Move(-40, 0);},
            () -> {robot.movement.Move(-40,90);},
            () -> {robot.movement.Move(-220,90);},
    };
    Runnable[] downPosition = {

            () -> { robot.movement.Move(-59, -30);
                robot.lift.setElevatorTarget(Lift.ElevatorPosition.UP);},
            () -> robot.bucket.setBucketPosition(Bucket.BucketPosition.EJECT),
            () -> robot.bucket.setBucketPosition(Bucket.BucketPosition.COLLECT),
            () -> {robot.lift.setElevatorTarget(Lift.ElevatorPosition.DOWN); },
            () -> {robot.movement.Move(-40, -30);},
            () -> {robot.movement.Move(-40,90);},
            () -> {robot.movement.Move(-220,90);},
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
        robot.duck.redOrBlue(PositionOnField.RED, PositionToSearch.LEFT);
        super.runOpMode();
    }
}