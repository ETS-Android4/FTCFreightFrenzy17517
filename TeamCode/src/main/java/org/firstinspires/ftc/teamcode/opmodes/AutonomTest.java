package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.teamcode.VariablesDashboard.MovementConfig.angle;
import static org.firstinspires.ftc.teamcode.VariablesDashboard.MovementConfig.dist;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.robot.Bucket;
import org.firstinspires.ftc.teamcode.robot.Lift;

import java.nio.ReadOnlyBufferException;

@Autonomous
public class AutonomTest extends BaseAutonomous {

    Runnable[] test = {
            () -> {robot.movement.Move(-62, -37);                                //-64, -37
                robot.lift.setElevatorTarget(Lift.ElevatorPosition.UP);},
            () -> robot.bucket.setBucketPosition(Bucket.BucketPosition.EJECT),
            () -> robot.bucket.setBucketPosition(Bucket.BucketPosition.COLLECT),
            () -> robot.lift.setElevatorTarget(Lift.ElevatorPosition.DOWN),
            () -> robot.movement.Move(-50, 0),
            () -> robot.movement.Move(-50,90),
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
            () -> robot.movement.Move(-262,0),
            () -> robot.bucket.setBucketPosition(Bucket.BucketPosition.EJECT),
            () -> robot.bucket.setBucketPosition(Bucket.BucketPosition.COLLECT),
            () -> robot.lift.setElevatorTarget(Lift.ElevatorPosition.DOWN),
            () -> robot.movement.Move(-230,-90),
            () -> robot.movement.Move(-380,-90),
    };

    @Override
    public void main() {
            execute(test);
    }

}