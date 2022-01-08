package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.misc.PositionOnField;
import org.firstinspires.ftc.teamcode.robot.Bucket;
import org.firstinspires.ftc.teamcode.robot.Lift;

@Autonomous
public class AutonomBlueTeam extends BaseDetectionAutonomous {

    Runnable downPosition[] = {

            () -> robot.movement.Move(-35, 27),
            () -> robot.movement.Move(-50, 27),
            () -> robot.lift.setElevatorTarget(Lift.ElevatorPosition.DOWN),
            () -> robot.bucket.setBucketPosition(Bucket.BucketPosition.COLLECT),
            () -> sleep(1900),
            () -> robot.movement.Move(-30),

            () -> robot.movement.Move(-30, 110),
            () -> robot.movement.Move(90, 110, 1.5)

/*
            () -> {RobotModules.movement.Move(0,90);},
            () -> {RobotModules.movement.Move(-100, 90);},
            () -> {RobotModules.movement.Move(-110,130);},
            () -> {RobotModules.duck.DuckSpin(true);},
            () -> {RobotModules.movement.Move(-90,90);},
            () -> {RobotModules.movement.Move(140,90);}
*/
    };
    Runnable[] middlePosition = {
            () -> robot.movement.Move(-35, 27),
            () -> {
                robot.movement.Move(-50, 27, 0.6);
                robot.lift.setElevatorTarget(Lift.ElevatorPosition.MIDDLE);},
            () -> robot.bucket.setBucketPosition(Bucket.BucketPosition.EJECT),
            () -> robot.bucket.setBucketPosition(Bucket.BucketPosition.COLLECT),
            () -> robot.lift.setElevatorTarget(Lift.ElevatorPosition.DOWN),
            () -> robot.movement.Move(-40),

            () -> robot.movement.Move(-40, 90),
            () -> robot.movement.Move(70, 90, 1.5)
/*
            () -> {RobotModules.movement.Move(0,90);},
            () -> {RobotModules.movement.Move(-100, 90);},
            () -> {RobotModules.movement.Move(-110,130);},
            () -> {RobotModules.duck.DuckSpin(true);},
            () -> {RobotModules.movement.Move(-90,90);},
            () -> {RobotModules.movement.Move(140,90);}
*/
    };
    Runnable[] upPosition = {
            () -> robot.movement.Move(-35, 27),
            () -> {
                robot.movement.Move(-54, 27, 0.6);
                robot.lift.setElevatorTarget(Lift.ElevatorPosition.UP);},
            () -> robot.bucket.setBucketPosition(Bucket.BucketPosition.EJECT),
            () -> robot.bucket.setBucketPosition(Bucket.BucketPosition.COLLECT),
            () -> robot.lift.setElevatorTarget(Lift.ElevatorPosition.DOWN),
            () -> robot.movement.Move(-40),

            () -> robot.movement.Move(-40, 90),
            () -> robot.movement.Move(70, 90, 1.5)
/*
            () -> {RobotModules.movement.Move(0,90);},
            () -> {RobotModules.movement.Move(-100, 90);},
            () -> {RobotModules.movement.Move(-110,130);},
            () -> {RobotModules.duck.DuckSpin(true);},
            () -> {RobotModules.movement.Move(-90,90);},
            () -> {RobotModules.movement.Move(140,90);}
*/
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
        robot.duck.redOrBlue(PositionOnField.BLUE);
        super.runOpMode();
    }
}