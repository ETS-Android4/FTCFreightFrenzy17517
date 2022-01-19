package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.teamcode.VariablesDashboard.MovementConfig.angle;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.robot.Bucket;

@Autonomous
public class AutonomTest extends BaseAutonomous {

    Runnable[] test = {
            ()-> {robot.movement.Move(0,angle);
                robot.timer.delay(100);
                robot.movement.teleometryEncoder();},
            ()-> {robot.movement.Move(0,0);},
            () -> {robot.movement.teleometryEncoder();}
    };

    @Override
    public void main(){
        while (opModeIsActive()) {
            execute(test);
        }
    }

}