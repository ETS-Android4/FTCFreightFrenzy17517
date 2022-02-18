package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.robot.Bucket;
import org.firstinspires.ftc.teamcode.robot.Lift;

@Autonomous
public class AutonomTest extends BaseAutonomous {

    Runnable[] test = {() -> { robot.movement.Move(-58, -37);                                //-64, -37
        robot.lift.setElevatorTarget(Lift.ElevatorPosition.UP);},
            () -> robot.bucket.setBucketPosition(Bucket.BucketPosition.EJECT),
            () -> robot.bucket.setBucketPosition(Bucket.BucketPosition.COLLECT),
            () -> {robot.lift.setElevatorTarget(Lift.ElevatorPosition.DOWN); },
            () -> {robot.movement.Move(-20, -37);},
            () -> {robot.movement.Move(-20, 90);},
            () -> {robot.movement.Move(-105, 90);},
            () -> robot.movement.Move(-105,180),
            () -> robot.movement.Move(-108,180),
            () -> {robot.duck.duckSpin(true);
                robot.movement.Move(-108,180);},
            () -> robot.movement.Move(-75,180),
            () -> robot.movement.Move(-75,90),
            () -> robot.movement.Move(150,90, 1.4),

    };

    @Override
    public void main() {

            execute(test);

    }
}
