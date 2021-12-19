package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.VariablesDashboard;
import org.firstinspires.ftc.teamcode.robot.Duck;
import org.firstinspires.ftc.teamcode.robot.Elevator;
import org.firstinspires.ftc.teamcode.robot.RobotModules;

@Autonomous
public class AutonomTest extends BaseAutonomous {

    Runnable[] test ={
            () -> {dashboard.getTelemetry().addData("pos", RobotModules.arucoDetect.getPosition());},
            () -> {dashboard.getTelemetry().addData("timePosition", RobotModules.arucoDetect.timePosition);},
            () -> {dashboard.getTelemetry().update();},

            () -> {sleep(100000);}
    };

    @Override
    protected Runnable[] getUpPosition() {
        return test;
    }

    @Override
    protected Runnable[] getMiddlePosition() {
        return test;
    }

    @Override
    protected Runnable[] getDownPosition() {
        return test;
    }

    @Override
    public void runOpMode(){

        super.runOpMode();
    }
}