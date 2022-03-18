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
                robot.movement.Move(-62, 40);
                robot.lift.setElevatorTarget(Lift.ElevatorPosition.DOWN);
            },
            () -> robot.bucket.setBucketPosition(Bucket.BucketPosition.EJECT),
            () -> robot.bucket.setBucketPosition(Bucket.BucketPosition.COLLECT),
            () -> robot.lift.setElevatorTarget(Lift.ElevatorPosition.DOWN),
            () -> robot.movement.Move(-34.92, 34),
            () -> robot.movement.Move(-34.92, 90),
            () -> robot.movement.Move(103.168, 90, 1.6),
    };
    Runnable[] middlePosition = {


            () -> {
                robot.movement.Move(-61.72, 34);
                robot.lift.setElevatorTarget(Lift.ElevatorPosition.MIDDLE);
            },
            () -> robot.bucket.setBucketPosition(Bucket.BucketPosition.EJECT),
            () -> robot.bucket.setBucketPosition(Bucket.BucketPosition.COLLECT),
            () -> robot.lift.setElevatorTarget(Lift.ElevatorPosition.DOWN),
            () -> robot.movement.Move(-34.92, 34),
            () -> robot.movement.Move(-34.92, 90),
            () -> robot.movement.Move(103.168, 90, 1.6),
    };
    Runnable[] upPosition = {

            () -> {
                robot.movement.Move(-60, 40);
                robot.lift.setElevatorTarget(Lift.ElevatorPosition.UP);
            },
            () -> robot.movement.Move(-67,40,1.5),
            () -> {
                robot.bucket.setBucketPosition(Bucket.BucketPosition.EJECT);
                robot.movement.Move(-72, 40, 0.5);
            },
            () -> robot.bucket.setBucketPosition(Bucket.BucketPosition.COLLECT),
            () -> robot.lift.setElevatorTarget(Lift.ElevatorPosition.DOWN),
            () -> robot.movement.Move(-45, 40),
            () -> robot.movement.Move(-45, 75),
            () -> robot.timer.delay(2),
            () -> robot.movement.Move(90, 60, 1.6),
            () -> robot.movement.Move(105, 45)
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
        robot.duck.setFieldPosition(AllianceColor.BLUE, StartingPosition.LEFT);
        super.runOpMode();
    }
}