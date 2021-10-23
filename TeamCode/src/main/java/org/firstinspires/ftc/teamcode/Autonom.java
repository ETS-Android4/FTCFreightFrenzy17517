package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.VariablesDashboard.ManipulatorConfig.*;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.opmodes.RobotModules;

@Autonomous
public class Autonom extends LinearOpMode {

    FtcDashboard dashboard;

    //creating variables

    private final RobotModules robotModules = new RobotModules(this);

    @Override
    public void runOpMode() {

        dashboard = FtcDashboard.getInstance();

        robotModules.init();

        waitForStart();
        {

        }
    }
}