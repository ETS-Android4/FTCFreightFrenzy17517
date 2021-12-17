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
public class AutonomBlueTeam extends LinearOpMode {

    FtcDashboard dashboard;

    private final RobotModules robotModules = new RobotModules(this);
    private VariablesDashboard vb = new VariablesDashboard();

    Runnable actions[] = {
            () -> {RobotModules.duck.redOrBlue(Duck.PositionOnField.BLUE);},
            () -> {RobotModules.movement.Move(-38);
                   RobotModules.elevator.ElevatorPosition(Elevator.ElevatorPosition.UP);},
            () -> {RobotModules.elevator.MoveServoForElevator(true);},
            () -> {RobotModules.elevator.MoveServoForElevator(false);},
            () -> {RobotModules.elevator.ElevatorPosition(Elevator.ElevatorPosition.DOWN);},
            () -> {RobotModules.movement.Move(-10);},
            () -> {RobotModules.movement.Move(0,90);},
            () -> {RobotModules.movement.resetEncoders(DcMotor.RunMode.RUN_WITHOUT_ENCODER);},
            () -> {RobotModules.movement.Move(-83,120);},
            () -> {RobotModules.duck.DuckSpin(true);},
            () -> {RobotModules.movement.Move(140,130);}
    };
    Runnable actions2[] ={
            () -> {RobotModules.duck.redOrBlue(Duck.PositionOnField.BLUE);},
            () -> {RobotModules.movement.Move(-20,0);},
            () -> {RobotModules.movement.Move(-20,180);},
            () -> {RobotModules.movement.Move(-20,165);},
            () -> {RobotModules.movement.Move(-63,165);},
            () -> {RobotModules.duck.DuckSpin(true);},
            () -> {RobotModules.movement.Move(-27,180);},
    };
    Runnable actions3[] ={
            () -> {RobotModules.duck.redOrBlue(Duck.PositionOnField.BLUE);},
            () -> {RobotModules.movement.Move(-40);
                RobotModules.elevator.ElevatorPosition(Elevator.ElevatorPosition.UP);},
            () -> {RobotModules.elevator.MoveServoForElevator(true);},
            () -> {RobotModules.elevator.MoveServoForElevator(false);},
            () -> {RobotModules.elevator.ElevatorPosition(Elevator.ElevatorPosition.DOWN);},
            () -> {RobotModules.movement.Move(-30,0);},
            () -> {RobotModules.movement.Move(-30,90);},
            () -> {RobotModules.movement.Move(120,90);},
             //      RobotModules.brush.brushMotorMove(true);},
           // () -> {RobotModules.movement.Move(-20,90);},
//            () -> {RobotModules.movement.Move(-25,0);
 //               RobotModules.elevator.ElevatorPosition(Elevator.ElevatorPosition.UP);},
 //           () -> {RobotModules.elevator.MoveServoForElevator(true);},
 //           () -> {RobotModules.elevator.MoveServoForElevator(false);},
  //          () -> {RobotModules.elevator.ElevatorPosition(Elevator.ElevatorPosition.DOWN);},
   //         () -> {RobotModules.movement.Move(-30,0);},
    //        () -> {RobotModules.movement.Move(-20,90);},
     //       () -> {RobotModules.movement.Move(100,90);},
    };
    Runnable actions4[]={
            ()->{RobotModules.brush.brushMotorMove(true);},
            ()->{RobotModules.movement.Move(100);},
            ()->{RobotModules.brush.brushMotorMove(false);},
            ()->{RobotModules.elevator.ElevatorPosition(Elevator.ElevatorPosition.MIDDLE);},
            ()->{RobotModules.elevator.MoveServoForElevator(true);},
            ()->{RobotModules.elevator.MoveServoForElevator(false);}
    };
    public int queue = 0;
    @Override
    public void runOpMode() {

        dashboard = FtcDashboard.getInstance();

        robotModules.init();
        RobotModules.duck.redOrBlue(Duck.PositionOnField.BLUE);
        waitForStart();
        queue = 0;
        while(opModeIsActive() && queue < actions4.length) {
            Runnable action = actions4[queue];
            action.run();
            robotModules.update();
            if (robotModules.line()) {
                queue++;
            }
        }
    }
}