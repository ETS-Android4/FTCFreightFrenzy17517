package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.VariablesDashboard.ManipulatorConfig.*;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import static org.firstinspires.ftc.teamcode.VariablesDashboard.MovementConfig.*;
import org.firstinspires.ftc.teamcode.opmodes.RobotModules;

@Autonomous
public class AutonomBlueTeam extends LinearOpMode {

    FtcDashboard dashboard;

    //creating variables

    private final RobotModules robotModules = new RobotModules(this);
    private VariablesDashboard vb = new VariablesDashboard();

    @Override
    public void runOpMode() {

        dashboard = FtcDashboard.getInstance();

        robotModules.init();

        waitForStart();
        {

            robotModules.movement.Move(-37.5 ,-40);
            robotModules.movement.Move(-45,-30);
            robotModules.duck.setDirection(-1);
            robotModules.duck.DuckSpin(true);
            sleep(7000);
            robotModules.movement.Move(-60,-90);

        }
    }
}