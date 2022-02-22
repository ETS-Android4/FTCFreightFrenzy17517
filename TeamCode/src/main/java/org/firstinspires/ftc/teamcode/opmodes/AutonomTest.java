package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.robot.Bucket;
import org.firstinspires.ftc.teamcode.robot.Lift;

@Autonomous
public class AutonomTest extends BaseAutonomous {

    Runnable[] test = {                         //-64, -37
        () -> { robot.movement.Move(-58, -34.5);                                //-64, -37
            robot.lift.setElevatorTarget(Lift.ElevatorPosition.DOWN);},
        () -> robot.bucket.setBucketPosition(Bucket.BucketPosition.EJECT),
        () -> robot.bucket.setBucketPosition(Bucket.BucketPosition.COLLECT),
        () -> {robot.lift.setElevatorTarget(Lift.ElevatorPosition.DOWN); },
        () -> robot.movement.Move(-10,0),

    };

    @Override
    public void main() {

            execute(test);

    }
}
