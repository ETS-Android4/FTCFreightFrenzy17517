package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.R;
import org.firstinspires.ftc.teamcode.VariablesDashboard;
import org.firstinspires.ftc.teamcode.VariablesDashboard.MovementConfig.*;
import org.firstinspires.ftc.teamcode.robot.Duck;
import org.firstinspires.ftc.teamcode.robot.Elevator;
import org.firstinspires.ftc.teamcode.robot.RobotModules;

@Autonomous
public class AutonomRedTeam extends BaseAutonomous {

    Runnable[] downPosition = {

            () -> {dashboard.getTelemetry().addData("qq", RobotModules.arucoDetect.getPosition());
                dashboard.getTelemetry().update();},
            () -> {RobotModules.elevator.ElevatorPosition(Elevator.ElevatorPosition.DOWN);
                RobotModules.movement.Move(-50, -27, 0.5);},
            () -> {RobotModules.elevator.MoveServoForElevator(true);},
            () -> {sleep(100);},
            () -> {RobotModules.elevator.MoveServoForElevator(false);},
            () -> {RobotModules.elevator.ElevatorPosition(Elevator.ElevatorPosition.DOWN);},
            () -> {RobotModules.movement.Move(-30,-90);},
            () -> {RobotModules.movement.Move(80,-90);},

    };
    Runnable[] middlePosition ={

            () -> {dashboard.getTelemetry().addData("qq", RobotModules.arucoDetect.getPosition());
                dashboard.getTelemetry().update();},
            () -> {RobotModules.elevator.ElevatorPosition(Elevator.ElevatorPosition.MIDDLE);
                RobotModules.movement.Move(-50, -27,0.5);},
            () -> {RobotModules.elevator.MoveServoForElevator(true);},
            () -> {RobotModules.elevator.MoveServoForElevator(false);},
            () -> {RobotModules.elevator.ElevatorPosition(Elevator.ElevatorPosition.DOWN);},
            () -> {RobotModules.movement.Move(-30,-90);},
            () -> {RobotModules.movement.Move(80,-90);},

    };
    Runnable[] upPosition ={


            () -> {dashboard.getTelemetry().addData("qq", RobotModules.arucoDetect.getPosition());
                dashboard.getTelemetry().update();},
            () -> {RobotModules.elevator.ElevatorPosition(Elevator.ElevatorPosition.UP);
                RobotModules.movement.Move(-50,-27,0.5);},
            () -> {RobotModules.elevator.MoveServoForElevator(true);},
            () -> {RobotModules.elevator.MoveServoForElevator(false);},
            () -> {RobotModules.elevator.ElevatorPosition(Elevator.ElevatorPosition.DOWN);},

            () -> {RobotModules.movement.Move(-30,-90);},
            () -> {RobotModules.movement.Move(80,-90);},

    };

    @Override
    protected Runnable[] getUpPosition() {
        return upPosition;
    }

    @Override
    protected Runnable[] getMiddlePosition() {
        return middlePosition;
    }

    @Override
    protected Runnable[] getDownPosition() {
        return downPosition;
    }

    @Override
    public void runOpMode(){
        RobotModules.duck.redOrBlue(Duck.PositionOnField.RED);
        super.runOpMode();
    }
}