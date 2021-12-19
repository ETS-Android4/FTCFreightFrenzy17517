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
public class AutonomBlueTeam extends BaseAutonomous {

    Runnable downPosition[] = {
            () -> {dashboard.getTelemetry().addData("qq", RobotModules.arucoDetect.getPosition());
                dashboard.getTelemetry().update();},
            () -> {RobotModules.movement.Move(-45,40);},
            () -> {RobotModules.elevator.MoveServoForElevator(true);},
            () -> {RobotModules.elevator.MoveServoForElevator(false);},
            () -> {RobotModules.movement.Move(-10);},
            () -> {RobotModules.movement.Move(0,90);},
            () -> {RobotModules.movement.resetEncoders(DcMotor.RunMode.RUN_WITHOUT_ENCODER);},
            () -> {RobotModules.movement.Move(-83,120);},
            () -> {RobotModules.duck.DuckSpin(true);},
            () -> {RobotModules.movement.Move(140,130);}
    };
    Runnable[] middlePosition ={
            () -> {dashboard.getTelemetry().addData("qq", RobotModules.arucoDetect.getPosition());
                    dashboard.getTelemetry().update();},
            () -> {RobotModules.elevator.ElevatorPosition(Elevator.ElevatorPosition.MIDDLE);
                RobotModules.movement.Move(-48,27 );},
            () -> {RobotModules.elevator.MoveServoForElevator(true);},
            () -> {RobotModules.elevator.MoveServoForElevator(false);},
            () -> {RobotModules.elevator.ElevatorPosition(Elevator.ElevatorPosition.DOWN);},
            () -> {RobotModules.movement.Move(-10);},
            () -> {RobotModules.movement.Move(0,90);},
            () -> {RobotModules.movement.resetEncoders(DcMotor.RunMode.RUN_WITHOUT_ENCODER);},
            () -> {RobotModules.movement.Move(-95,100);},
            () -> {RobotModules.movement.Move(-110,150);},
            () -> {RobotModules.duck.DuckSpin(true);},
            () -> {RobotModules.movement.Move(-90,90);},
            () -> {RobotModules.movement.Move(100,70);}
    };
    Runnable[] upPosition ={

            () -> {dashboard.getTelemetry().addData("qq", RobotModules.arucoDetect.getPosition());
                dashboard.getTelemetry().update();},
            () -> {RobotModules.movement.Move(-45,40);
                RobotModules.elevator.ElevatorPosition(Elevator.ElevatorPosition.UP);},
            () -> {RobotModules.elevator.MoveServoForElevator(true);},
            () -> {RobotModules.elevator.MoveServoForElevator(false);},
            () -> {RobotModules.elevator.ElevatorPosition(Elevator.ElevatorPosition.DOWN);},
            () -> {RobotModules.movement.Move(-10);},
            () -> {RobotModules.movement.Move(0,90);},
            () -> {RobotModules.movement.resetEncoders(DcMotor.RunMode.RUN_WITHOUT_ENCODER);},
            () -> {RobotModules.movement.Move(-8,120);},
            () -> {RobotModules.duck.DuckSpin(true);},
            () -> {RobotModules.movement.Move(140,130);}

    };
    Runnable[] test ={
            () -> {dashboard.getTelemetry().addData("pos", RobotModules.arucoDetect.getPosition());},
            () -> {dashboard.getTelemetry().addData("timePosition", RobotModules.arucoDetect.timePosition);},
            () -> {dashboard.getTelemetry().update();},
            () -> {sleep(100000);}
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
        RobotModules.duck.redOrBlue(Duck.PositionOnField.BLUE);
        super.runOpMode();
    }
}