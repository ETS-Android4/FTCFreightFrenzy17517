package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.misc.Pose2D;
import org.firstinspires.ftc.teamcode.robot.Bucket;
import org.firstinspires.ftc.teamcode.robot.Lift;

@Autonomous
public class AutonomTest extends BaseAutonomous {

    Runnable[] test = {                         //-64, -37
            () -> {
                robot.odometry.setCurrentPosition(new Pose2D(0,0,0));
                robot.globalMovement.Move(30, 0);
            },
    };

    @Override
    public void main() {

        execute(test);

    }
}
