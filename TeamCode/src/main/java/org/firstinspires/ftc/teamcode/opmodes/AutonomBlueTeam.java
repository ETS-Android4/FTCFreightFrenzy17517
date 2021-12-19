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
            () -> {RobotModules.movement.Move(-35,27);},
            () -> {RobotModules.movement.Move(-45,27);},
            () -> {RobotModules.elevator.MoveServoForElevator(true);
                    sleep(1000);},
            () -> {RobotModules.elevator.MoveServoForElevator(false);},
            () -> {RobotModules.movement.Move(-30);},

            //() -> {RobotModules.movement.Move(-30,90);},
            //() -> {RobotModules.movement.Move(70,90,1.5);}


            () -> {RobotModules.movement.Move(0,90);},
            () -> {RobotModules.movement.Move(-100, 90);},
            () -> {RobotModules.movement.Move(-110,130);},
            () -> {RobotModules.duck.DuckSpin(true);},
            () -> {RobotModules.movement.Move(-90,90);},
            () -> {RobotModules.movement.Move(140,90);}

    };
    Runnable[] middlePosition ={
            () -> {dashboard.getTelemetry().addData("qq", RobotModules.arucoDetect.getPosition());
                    dashboard.getTelemetry().update();},
            () -> {RobotModules.movement.Move(-35,27);},
            () -> {RobotModules.movement.Move(-45,27, 0.6);
                    RobotModules.elevator.ElevatorPosition(Elevator.ElevatorPosition.MIDDLE);},
            () -> {RobotModules.elevator.MoveServoForElevator(true);
                    sleep(1000);},
            () -> {RobotModules.elevator.MoveServoForElevator(false);},
            () -> {RobotModules.elevator.ElevatorPosition(Elevator.ElevatorPosition.DOWN);},
            () -> {RobotModules.movement.Move(-30);},

            //() -> {RobotModules.movement.Move(-30,90);},
            //() -> {RobotModules.movement.Move(70,90,1.5);}

            () -> {RobotModules.movement.Move(0,90);},
            () -> {RobotModules.movement.Move(-100, 90);},
            () -> {RobotModules.movement.Move(-110,130);},
            () -> {RobotModules.duck.DuckSpin(true);},
            () -> {RobotModules.movement.Move(-90,90);},
            () -> {RobotModules.movement.Move(140,90);}

    };
    Runnable[] upPosition ={

            () -> {dashboard.getTelemetry().addData("qq", RobotModules.arucoDetect.getPosition());
                    dashboard.getTelemetry().update();},
            () -> {RobotModules.movement.Move(-35,27);},
            () -> {RobotModules.movement.Move(-65,27, 0.6);
                RobotModules.elevator.ElevatorPosition(Elevator.ElevatorPosition.UP);},
            () -> {RobotModules.elevator.MoveServoForElevator(true);},
            () -> {RobotModules.elevator.MoveServoForElevator(false);},
            () -> {RobotModules.elevator.ElevatorPosition(Elevator.ElevatorPosition.DOWN);},
            () -> {RobotModules.movement.Move(-30);},

   //         () -> {RobotModules.movement.Move(-30,90);},
  //          () -> {RobotModules.movement.Move(70,90, 1.5);}

            () -> {RobotModules.movement.Move(0,90);},
            () -> {RobotModules.movement.Move(-100, 90);},
            () -> {RobotModules.movement.Move(-110,130);},
            () -> {RobotModules.duck.DuckSpin(true);},
            () -> {RobotModules.movement.Move(-90,90);},
            () -> {RobotModules.movement.Move(140,90);}

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