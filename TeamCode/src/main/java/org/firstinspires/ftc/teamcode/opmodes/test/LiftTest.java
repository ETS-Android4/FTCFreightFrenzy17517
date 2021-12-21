package org.firstinspires.ftc.teamcode.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opmodes.BaseOpMode;
import org.firstinspires.ftc.teamcode.robot.Lift;

@Autonomous
public class LiftTest extends BaseOpMode {

    @Override
    public void startLoop() {
        telemetry.addData("Test", "Lift will self-test 5 seconds after Start button has been pressed.");
        telemetry.update();
    }

    @Override
    public void main() {
        robot.timer.delay(5);
        while (!robot.timer.actionIsCompleted() && opModeIsActive()) {
            telemetry.addData("Test", "Running...");
            telemetry.addData("Lift position", robot.lift.getElevatorPosition());
            telemetry.addData("Time before test", robot.timer.getTimeLeft());
            telemetry.update();
        }
        robot.lift.setElevatorTarget(Lift.ElevatorPosition.MIDDLE);
        while (!robot.lift.actionIsCompleted() && opModeIsActive()) {
            telemetry.addData("Test", "Running...");
            telemetry.addData("Lift target", robot.lift.getElevatorPosition());
            telemetry.addData("Lift encoder", robot.lift.getLiftEncoderPosition());
            robot.update();
            telemetry.update();
        }
        robot.lift.setElevatorTarget(Lift.ElevatorPosition.DOWN);
        while (!robot.lift.actionIsCompleted() && opModeIsActive()) {
            telemetry.addData("Test", "Running...");
            telemetry.addData("Lift target", robot.lift.getElevatorPosition());
            telemetry.addData("Lift encoder", robot.lift.getLiftEncoderPosition());
            telemetry.update();
        }
        robot.lift.setElevatorTarget(Lift.ElevatorPosition.UP);
        while (!robot.lift.actionIsCompleted() && opModeIsActive()) {
            telemetry.addData("Test", "Running...");
            telemetry.addData("Lift target", robot.lift.getElevatorPosition());
            telemetry.addData("Lift encoder", robot.lift.getLiftEncoderPosition());
            telemetry.update();
        }
        robot.lift.setElevatorTarget(Lift.ElevatorPosition.DOWN);
        while (!robot.lift.actionIsCompleted() && opModeIsActive()) {
            telemetry.addData("Test", "Running...");
            telemetry.addData("Lift target", robot.lift.getElevatorPosition());
            telemetry.addData("Lift encoder", robot.lift.getLiftEncoderPosition());
            telemetry.update();
        }
    }
}