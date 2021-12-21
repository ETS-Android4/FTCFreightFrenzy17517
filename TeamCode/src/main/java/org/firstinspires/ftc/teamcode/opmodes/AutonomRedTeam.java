package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.misc.PositionOnField;
import org.firstinspires.ftc.teamcode.robot.Bucket;
import org.firstinspires.ftc.teamcode.robot.Lift;

@Autonomous
public class AutonomRedTeam extends BaseAutonomous {

    Runnable[] downPosition = {

            () -> {
                telemetry.addData("qq", robot.arucoDetect.getPosition());
                telemetry.update();
            },
            () -> {
                robot.lift.setElevatorTarget(Lift.ElevatorPosition.DOWN);
                robot.movement.Move(-55, -27, 0.5);
            },
            () -> robot.bucket.setBucketPosition(Bucket.BucketPosition.EJECT),
            () -> sleep(100),
            () -> robot.bucket.setBucketPosition(Bucket.BucketPosition.COLLECT),
            () -> robot.lift.setElevatorTarget(Lift.ElevatorPosition.DOWN),
            () -> robot.movement.Move(-20, 0),
            () -> robot.movement.Move(-20, -90),
            () -> robot.movement.Move(40, -90),

    };
    Runnable[] middlePosition = {

            () -> {
                telemetry.addData("qq", robot.arucoDetect.getPosition());
                telemetry.update();
            },
            () -> {
                robot.lift.setElevatorTarget(Lift.ElevatorPosition.MIDDLE);
                robot.movement.Move(-55, -27, 0.5);
            },
            () -> robot.bucket.setBucketPosition(Bucket.BucketPosition.EJECT),
            () -> robot.bucket.setBucketPosition(Bucket.BucketPosition.COLLECT),
            () -> robot.lift.setElevatorTarget(Lift.ElevatorPosition.DOWN),
            () -> robot.movement.Move(-20, 0),
            () -> robot.movement.Move(-20, -90),
            () -> robot.movement.Move(40, -90),

    };
    Runnable[] upPosition = {

            () -> {
                telemetry.addData("qq", robot.arucoDetect.getPosition());
                telemetry.update();
            },
            () -> {
                robot.lift.setElevatorTarget(Lift.ElevatorPosition.UP);
                robot.movement.Move(-55, -27, 0.5);
            },
            () -> robot.bucket.setBucketPosition(Bucket.BucketPosition.EJECT),
            () -> robot.bucket.setBucketPosition(Bucket.BucketPosition.COLLECT),
            () -> robot.lift.setElevatorTarget(Lift.ElevatorPosition.DOWN),
            () -> robot.movement.Move(-20, 0),
            () -> robot.movement.Move(-20, -90),
            () -> robot.movement.Move(40, -90),

    };

    @Override
    protected Runnable[] getUpPosition() {
        return upPosition;
    }

    @Override
    protected Runnable[] getMiddlePosition() {
        return middlePosition;
    }

    @Override
    protected Runnable[] getDownPosition() {
        return downPosition;
    }

    @Override
    public void runOpMode() {
        robot.duck.redOrBlue(PositionOnField.RED);
        super.runOpMode();
    }
}