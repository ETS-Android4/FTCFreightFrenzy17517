package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.VariablesDashboard;
import org.firstinspires.ftc.teamcode.robot.RobotModules;

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
            robotModules.movement.Move(-38 ,-40);
            robotModules.movement.Move(-45,-22);
            robotModules.duck.setDirection(-1);
            robotModules.duck.DuckSpin(true);
            sleep(4500);
            robotModules.movement.Move(-45,-80);
            robotModules.movement.Move(-85,-80);
            robotModules.movement.Move(-100,-95);
            //robotModules.movement.Move(56,-90);
            //robotModules.movement.Move(120, 90);
        }
    }
}