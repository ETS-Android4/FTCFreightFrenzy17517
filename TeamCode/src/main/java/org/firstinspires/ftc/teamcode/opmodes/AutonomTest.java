package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.teamcode.VariablesDashboard.Elevator.bucketServoDelay;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.robot.Duck;
import org.firstinspires.ftc.teamcode.robot.Elevator;
import org.firstinspires.ftc.teamcode.robot.RobotModules;

@Autonomous
public class AutonomTest extends BaseAutonomous {

    Runnable[] test ={
            () -> {RobotModules.elevator.MoveServoForElevator(true);},
            () -> {dashboard.getTelemetry().addData("posManip", RobotModules.elevator.motorLift.getCurrentPosition());},
            () -> {dashboard.getTelemetry().addData("bucketServoDelay", bucketServoDelay);},
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