package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.misc.PositionOnField;
import org.firstinspires.ftc.teamcode.misc.PositionToSearch;
import org.firstinspires.ftc.teamcode.robot.Bucket;
import org.firstinspires.ftc.teamcode.robot.Lift;

@Autonomous
public class AutonomRedTeamLeftPos extends BaseDetectionAutonomous {

    Runnable[] upPosition = {     //DOWN

            () -> { robot.movement.Move(-60, 25);                                //-64, -37
                robot.lift.setElevatorTarget(Lift.ElevatorPosition.DOWN);},
            () -> robot.bucket.setBucketPosition(Bucket.BucketPosition.EJECT),
            () -> robot.bucket.setBucketPosition(Bucket.BucketPosition.COLLECT),
            () -> {robot.lift.setElevatorTarget(Lift.ElevatorPosition.DOWN);
                robot.movement.Move(-5,37);},
            () -> robot.movement.Move(-5,-90),
            () -> robot.movement.Move(-95,-90),
            () -> {robot.duck.duckSpin(true);
                robot.movement.Move(-95,-90);},
            () -> robot.movement.Move(-70,-90),
            () -> robot.movement.Move(-25,-120),
            () -> robot.movement.Move(-15,-90),
            () -> robot.movement.Move(230,-90, 1.6),

    };
    Runnable[] middlePosition = {

            () -> { robot.movement.Move(-60, 25);                                //-64, -37
                robot.lift.setElevatorTarget(Lift.ElevatorPosition.MIDDLE);},
            () -> robot.bucket.setBucketPosition(Bucket.BucketPosition.EJECT),
            () -> robot.bucket.setBucketPosition(Bucket.BucketPosition.COLLECT),
            () -> {robot.lift.setElevatorTarget(Lift.ElevatorPosition.DOWN);
                robot.movement.Move(-5,37);},
            () -> robot.movement.Move(-5,-90),
            () -> robot.movement.Move(-95,-90),
            () -> {robot.duck.duckSpin(true);
                robot.movement.Move(-95,-90);},
            () -> robot.movement.Move(-70,-90),
            () -> robot.movement.Move(-25,-120),
            () -> robot.movement.Move(-15,-90),
            () -> robot.movement.Move(230,-90, 1.6),

    };
    Runnable[] downPosition = { //UP

            () -> {robot.movement.Move(-60, 25);
                   robot.lift.setElevatorTarget(Lift.ElevatorPosition.UP);},
            () -> robot.bucket.setBucketPosition(Bucket.BucketPosition.EJECT),
            () -> robot.bucket.setBucketPosition(Bucket.BucketPosition.COLLECT),
            () -> {robot.lift.setElevatorTarget(Lift.ElevatorPosition.DOWN);
                robot.movement.Move(-5,37);},
            () -> robot.movement.Move(-5,-90),
            () -> robot.movement.Move(-95,-90),
            () -> {robot.duck.duckSpin(true);
                robot.movement.Move(-95,-90);},
            () -> robot.movement.Move(-70,-90),
            () -> robot.movement.Move(-25,-120),
            () -> robot.movement.Move(-15,-90),
            () -> robot.movement.Move(230,-90, 1.6),


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
        robot.duck.redOrBlue(PositionOnField.RED, PositionToSearch.RIGHT);
        super.runOpMode();
    }
}