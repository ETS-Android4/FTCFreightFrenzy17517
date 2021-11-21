package org.firstinspires.ftc.teamcode.opmodes;

import android.drm.DrmStore;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.VariablesDashboard;
import org.firstinspires.ftc.teamcode.VariablesDashboard.MovementConfig.*;
import org.firstinspires.ftc.teamcode.robot.Duck;
import org.firstinspires.ftc.teamcode.robot.Elevator;
import org.firstinspires.ftc.teamcode.robot.RobotModules;

@Autonomous
public class AutonomRedTeam extends LinearOpMode {

    FtcDashboard dashboard;

    private final RobotModules robotModules = new RobotModules(this);
    private VariablesDashboard vb = new VariablesDashboard();

    Runnable actions[] = {      // элемент наверх уточка и парковка на складе
            () -> {RobotModules.duck.redOrBlue(Duck.PositionOnField.RED);},
            () -> {RobotModules.movement.Move(-36);
                   RobotModules.elevator.ElevatorPosition(Elevator.ElevatorPosition.UP);},
            () -> {RobotModules.elevator.MoveServoForElevator(true);},
            () -> {RobotModules.elevator.MoveServoForElevator(false);},
            () -> {RobotModules.elevator.ElevatorPosition(Elevator.ElevatorPosition.DOWN);},
            () -> {RobotModules.movement.Move(-10);},
            () -> {RobotModules.movement.Move(0,-90);},
            () -> {RobotModules.movement.resetEncoders(DcMotor.RunMode.RUN_WITHOUT_ENCODER);},
            () -> {RobotModules.movement.Move(-80,-90);},
            () -> {RobotModules.duck.DuckSpin(true);
                    RobotModules.movement.setMotorPowers(-0.1,0);},
            () -> {RobotModules.movement.resetEncoders(DcMotor.RunMode.RUN_WITHOUT_ENCODER);},
            () -> {RobotModules.movement.Move(230,-90);},

    };

    Runnable actions2[] = {    //элемент наверх и парковка на складе
            () -> {RobotModules.duck.redOrBlue(Duck.PositionOnField.RED);},
            () -> {RobotModules.movement.Move(-38);
                RobotModules.elevator.ElevatorPosition(Elevator.ElevatorPosition.UP);},
            () -> {RobotModules.elevator.MoveServoForElevator(true);},
            () -> {RobotModules.elevator.MoveServoForElevator(false);},
            () -> {RobotModules.elevator.ElevatorPosition(Elevator.ElevatorPosition.DOWN);},
            () -> {RobotModules.movement.Move(-23 );},
            () -> {RobotModules.movement.Move(0,90);},
            () -> {RobotModules.movement.resetEncoders(DcMotor.RunMode.RUN_WITHOUT_ENCODER);},
            () -> {RobotModules.movement.Move(-133,90);},
    };

    public int queue = 0;
    @Override
    public void runOpMode() {

        dashboard = FtcDashboard.getInstance();

        robotModules.init();

        waitForStart();
        queue = 0;
        while(opModeIsActive() && queue < actions.length) {
            Runnable action = actions[queue];
            action.run();
            robotModules.update();
            if (robotModules.line()) {
                queue++;
            }
        }
    }
}