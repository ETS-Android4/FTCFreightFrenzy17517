package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.VariablesDashboard.ManipulatorConfig.*;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import static org.firstinspires.ftc.teamcode.VariablesDashboard.MovementConfig.*;
import org.firstinspires.ftc.teamcode.opmodes.RobotModules;

@Autonomous
public class AutonomRedTeam extends LinearOpMode {

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
            robotModules.movement.Move(-47 ,-68);
            robotModules.duck.setDirection(1);
            robotModules.duck.DuckSpin(true);
            sleep(4200);
            robotModules.movement.Move(-40,-15);
            robotModules.movement.Move(60,-4);
            robotModules.movement.Move(230,0);
        }
    }
}