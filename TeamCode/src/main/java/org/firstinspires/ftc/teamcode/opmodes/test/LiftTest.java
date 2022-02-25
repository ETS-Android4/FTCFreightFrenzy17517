package org.firstinspires.ftc.teamcode.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opmodes.BaseOpMode;
import org.firstinspires.ftc.teamcode.robot.Lift;

@Autonomous
public class LiftTest extends BaseOpMode {

    @Override
    public void startLoop() {
        robot.telemetryNode.getTelemetry()
                .addData("Test", "Lift will self-test 5 seconds after Start button has been pressed.");
        robot.telemetryNode.getTelemetry().update();
    }

    @Override
    public void main() {
        robot.timer.delay(5);
        while (!robot.timer.actionIsCompleted() && opModeIsActive()) {
            robot.telemetryNode.getTelemetry().addData("Test", "Running...");
            robot.telemetryNode.getTelemetry().addData("Lift position", robot.lift.getElevatorPosition());
            robot.telemetryNode.getTelemetry().addData("Time before test", robot.timer.getTimeLeft());
            robot.update();
        }
        robot.lift.setElevatorTarget(Lift.ElevatorPosition.MIDDLE);
        while (!robot.lift.actionIsCompleted() && opModeIsActive()) {
            robot.telemetryNode.getTelemetry().addData("Test", "Running...");
            robot.telemetryNode.getTelemetry().addData("Lift target", robot.lift.getElevatorPosition());
            robot.telemetryNode.getTelemetry().addData("Lift encoder", robot.lift.getLiftEncoderPosition());
            robot.update();
        }
        robot.lift.setElevatorTarget(Lift.ElevatorPosition.DOWN);
        while (!robot.lift.actionIsCompleted() && opModeIsActive()) {
            robot.telemetryNode.getTelemetry().addData("Test", "Running...");
            robot.telemetryNode.getTelemetry().addData("Lift target", robot.lift.getElevatorPosition());
            robot.telemetryNode.getTelemetry().addData("Lift encoder", robot.lift.getLiftEncoderPosition());
            robot.update();
        }
        robot.lift.setElevatorTarget(Lift.ElevatorPosition.UP);
        while (!robot.lift.actionIsCompleted() && opModeIsActive()) {
            robot.telemetryNode.getTelemetry().addData("Test", "Running...");
            robot.telemetryNode.getTelemetry().addData("Lift target", robot.lift.getElevatorPosition());
            robot.telemetryNode.getTelemetry().addData("Lift encoder", robot.lift.getLiftEncoderPosition());
            robot.update();
        }
        robot.lift.setElevatorTarget(Lift.ElevatorPosition.DOWN);
        while (!robot.lift.actionIsCompleted() && opModeIsActive()) {
            robot.telemetryNode.getTelemetry().addData("Test", "Running...");
            robot.telemetryNode.getTelemetry().addData("Lift target", robot.lift.getElevatorPosition());
            robot.telemetryNode.getTelemetry().addData("Lift encoder", robot.lift.getLiftEncoderPosition());
            robot.update();
        }
    }
}