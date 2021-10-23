package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import static org.firstinspires.ftc.teamcode.VariablesDash.*;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.opmodes.Global;

@Autonomous
public class Autonom extends LinearOpMode {

    FtcDashboard dashboard;

    //creating variables
    private Movement movement = new Movement(this);
    private Manipulator manipulator = new Manipulator(this);
    private Global global = new Global(this);
    @Override
    public void runOpMode() {

        dashboard = FtcDashboard.getInstance();

        movement.init();
        manipulator.initManip();

        waitForStart();
        while (opModeIsActive()) {
            manipulator.MoveServo(posManip);
        }
    }
}