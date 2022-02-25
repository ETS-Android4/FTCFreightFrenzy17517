package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.sun.tools.javac.util.ArrayUtils;

import org.firstinspires.ftc.teamcode.misc.AllianceColor;
import org.firstinspires.ftc.teamcode.misc.StartingPosition;
import org.firstinspires.ftc.teamcode.robot.Bucket;
import org.firstinspires.ftc.teamcode.robot.Lift;

@Autonomous
public class AutonomRedTeamLeftPos extends BaseDetectionAutonomous {

    Runnable[] upPosition = {

            () -> {
                robot.movement.Move(-61.73, 25);
                robot.lift.setElevatorTarget(Lift.ElevatorPosition.UP);
            },
            () -> robot.bucket.setBucketPosition(Bucket.BucketPosition.EJECT),
            () -> robot.bucket.setBucketPosition(Bucket.BucketPosition.COLLECT),
            () -> {
                robot.lift.setElevatorTarget(Lift.ElevatorPosition.DOWN);
                robot.movement.Move(-5.14, 37);
            },
            () -> robot.movement.Move(-15.43, -90),
            () -> robot.movement.Move(-72.02, -90, 0.5),
            () -> {
                robot.duck.duckSpin(true);
                robot.movement.Move(-66.87, -90);
            },
            () -> robot.movement.Move(-30.86, -90),
            () -> robot.movement.Move(15.43, -120),
            () -> robot.movement.Move(20.58, -90),
            () -> robot.movement.Move(113.17, -90, 1),
            () -> robot.movement.Move(216.05, -90, 2),};
    Runnable[] middlePosition = {

            () -> {
                robot.movement.Move(-61.73, 25);                                //-64, -37
                robot.lift.setElevatorTarget(Lift.ElevatorPosition.MIDDLE);
            },
            () -> robot.bucket.setBucketPosition(Bucket.BucketPosition.EJECT),
            () -> robot.bucket.setBucketPosition(Bucket.BucketPosition.COLLECT),
            () -> {
                robot.lift.setElevatorTarget(Lift.ElevatorPosition.DOWN);
                robot.movement.Move(-5.14, 37);
            },
            () -> robot.movement.Move(-15.43, -90),
            () -> robot.movement.Move(-72.02, -90, 0.5),
            () -> {
                robot.duck.duckSpin(true);
                robot.movement.Move(-66.87, -90);
            },
            () -> robot.movement.Move(-30.86, -90),
            () -> robot.movement.Move(15.43, -120),
            () -> robot.movement.Move(20.58, -90),
            () -> robot.movement.Move(113.17, -90, 1),
            () -> robot.movement.Move(216.05, -90, 2),};
    Runnable[] downPosition = {

            () -> {
                robot.movement.Move(-61.73, 25);
                robot.lift.setElevatorTarget(Lift.ElevatorPosition.DOWN);
            },
            () -> robot.bucket.setBucketPosition(Bucket.BucketPosition.EJECT),
            () -> robot.bucket.setBucketPosition(Bucket.BucketPosition.COLLECT),
            () -> {
                robot.lift.setElevatorTarget(Lift.ElevatorPosition.DOWN);
                robot.movement.Move(-5.14, 37);
            },
            () -> robot.movement.Move(-15.43, -90),
            () -> robot.movement.Move(-72.02, -90, 0.5),
            () -> {
                robot.duck.duckSpin(true);
                robot.movement.Move(-66.87, -90);
            },
            () -> robot.movement.Move(-30.86, -90),
            () -> robot.movement.Move(15.43, -120),
            () -> robot.movement.Move(20.58, -90),
//            () -> robot.timer.delay(4),

            () -> robot.movement.Move(113.17, -90, 1),
            () -> robot.movement.Move(216.05, -90, 2),};

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