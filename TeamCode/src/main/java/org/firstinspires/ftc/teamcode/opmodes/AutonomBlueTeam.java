package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.misc.PositionOnField;
import org.firstinspires.ftc.teamcode.misc.PositionToSearch;
import org.firstinspires.ftc.teamcode.robot.Bucket;
import org.firstinspires.ftc.teamcode.robot.Lift;

@Autonomous
public class AutonomBlueTeam extends BaseDetectionAutonomous {

    Runnable[] downPosition = {

            () -> { robot.movement.Move(-61, -37);                                //-64, -37
                robot.lift.setElevatorTarget(Lift.ElevatorPosition.DOWN);},
            () -> robot.bucket.setBucketPosition(Bucket.BucketPosition.EJECT),
            () -> robot.bucket.setBucketPosition(Bucket.BucketPosition.COLLECT),
            () -> {robot.lift.setElevatorTarget(Lift.ElevatorPosition.DOWN); },
            () -> {robot.movement.Move(-50, 0);},
            () -> {robot.movement.Move(-50, 90);},
            () -> robot.movement.Move(-150,120),
            () -> robot.movement.Move(-150,150),
            () -> {robot.duck.duckSpin(true);
                robot.movement.Move(-155,150);},
            () -> robot.movement.Move(-120,150),
            () -> robot.movement.Move(-120,0),
            () -> {robot.brush.enableIntake(true);
                robot.movement.Move(-98,0);
                robot.timer.delay(2);},
            () -> robot.movement.Move(-120,0),
            () -> {robot.movement.Move(-120,-90);
                robot.brush.enableIntake(false);},
            () -> robot.movement.Move(-230,-90),
            () -> {robot.movement.Move(-230,0);
                robot.lift.setElevatorTarget(Lift.ElevatorPosition.UP); },
            () -> robot.movement.Move(-265,0),
            () -> robot.bucket.setBucketPosition(Bucket.BucketPosition.EJECT),
            () -> robot.bucket.setBucketPosition(Bucket.BucketPosition.COLLECT),
            () -> robot.lift.setElevatorTarget(Lift.ElevatorPosition.DOWN),
            () -> robot.movement.Move(-235,0),
            () -> robot.movement.Move(-235,-90),
            () -> robot.movement.Move(-420,-90),

    };
    Runnable[] middlePosition = {

            () -> { robot.movement.Move(-61, -37);                                //-64, -37
                robot.lift.setElevatorTarget(Lift.ElevatorPosition.MIDDLE);},
            () -> robot.bucket.setBucketPosition(Bucket.BucketPosition.EJECT),
            () -> robot.bucket.setBucketPosition(Bucket.BucketPosition.COLLECT),
            () -> {robot.lift.setElevatorTarget(Lift.ElevatorPosition.DOWN); },
            () -> {robot.movement.Move(-50, 0);},
            () -> {robot.movement.Move(-50, 90);},
            () -> robot.movement.Move(-150,120),
            () -> robot.movement.Move(-150,150),
            () -> {robot.duck.duckSpin(true);
                robot.movement.Move(-155,150);},
            () -> robot.movement.Move(-120,150),
            () -> robot.movement.Move(-120,0),
            () -> {robot.brush.enableIntake(true);
                robot.movement.Move(-98,0);
                robot.timer.delay(2);},
            () -> robot.movement.Move(-120,0),
            () -> {robot.movement.Move(-120,-90);
                robot.brush.enableIntake(false);},
            () -> robot.movement.Move(-230,-90),
            () -> {robot.movement.Move(-230,0);
                robot.lift.setElevatorTarget(Lift.ElevatorPosition.UP); },
            () -> robot.movement.Move(-265,0),
            () -> robot.bucket.setBucketPosition(Bucket.BucketPosition.EJECT),
            () -> robot.bucket.setBucketPosition(Bucket.BucketPosition.COLLECT),
            () -> robot.lift.setElevatorTarget(Lift.ElevatorPosition.DOWN),
            () -> robot.movement.Move(-235,0),
            () -> robot.movement.Move(-235,-90),
            () -> robot.movement.Move(-420,-90),

    };
    Runnable[] upPosition = {

            () -> { robot.movement.Move(-61, -37);                                //-64, -37
                robot.lift.setElevatorTarget(Lift.ElevatorPosition.UP);},
            () -> robot.bucket.setBucketPosition(Bucket.BucketPosition.EJECT),
            () -> robot.bucket.setBucketPosition(Bucket.BucketPosition.COLLECT),
            () -> {robot.lift.setElevatorTarget(Lift.ElevatorPosition.DOWN); },
            () -> {robot.movement.Move(-50, 0);},
            () -> {robot.movement.Move(-50, 90);},
            () -> robot.movement.Move(-150,120),
            () -> robot.movement.Move(-150,150),
            () -> {robot.duck.duckSpin(true);
                robot.movement.Move(-155,150);},
            () -> robot.movement.Move(-120,150),
            () -> robot.movement.Move(-120,0),
            () -> {robot.brush.enableIntake(true);
                robot.movement.Move(-98,0);
                robot.timer.delay(2);},
            () -> robot.movement.Move(-120,0),
            () -> {robot.movement.Move(-120,-90);
                robot.brush.enableIntake(false);},
            () -> robot.movement.Move(-230,-90),
            () -> {robot.movement.Move(-230,0);
                robot.lift.setElevatorTarget(Lift.ElevatorPosition.UP); },
            () -> robot.movement.Move(-265,0),
            () -> robot.bucket.setBucketPosition(Bucket.BucketPosition.EJECT),
            () -> robot.bucket.setBucketPosition(Bucket.BucketPosition.COLLECT),
            () -> robot.lift.setElevatorTarget(Lift.ElevatorPosition.DOWN),
            () -> robot.movement.Move(-235,0),
            () -> robot.movement.Move(-235,-90),
            () -> robot.movement.Move(-420,-90),

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
        robot.duck.redOrBlue(PositionOnField.BLUE, PositionToSearch.LEFT);
        super.runOpMode();
    }
}