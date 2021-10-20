package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.opmodes.Movement;

@Autonomous
public class Autonom extends LinearOpMode {

    FtcDashboard dashboard;

    //creating variables
    private Movement movement = new Movement(this);
    private Manipulator manipulator = new Manipulator(this);

    @Override
    public void runOpMode() {

        dashboard = FtcDashboard.getInstance();

        movement.init();

        waitForStart();
        while (opModeIsActive()){
            movement.Move(100);
            movement.Move(0);
        }
    }
}