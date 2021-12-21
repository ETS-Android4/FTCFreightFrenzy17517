package org.firstinspires.ftc.teamcode.opmodes.test;

import org.firstinspires.ftc.teamcode.opmodes.BaseOpMode;
import org.firstinspires.ftc.teamcode.robot.Lift;

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
            telemetry.addData("Lift position", robot.lift.getElevatorTarget());
            telemetry.addData("Time before test", robot.timer.getTimeLeft());
            telemetry.update();
        }
        robot.lift.setElevatorTarget(Lift.ElevatorPosition.MIDDLE);
        while (!robot.lift.actionIsCompleted() && opModeIsActive()) {
            telemetry.addData("Test", "Running...");
            telemetry.addData("Lift target", robot.lift.getElevatorTarget());
            telemetry.addData("Lift encoder", robot.lift.getLiftEncoderPosition());
            telemetry.update();
        }
        robot.lift.setElevatorTarget(Lift.ElevatorPosition.DOWN);
        while (!robot.lift.actionIsCompleted() && opModeIsActive()) {
            telemetry.addData("Test", "Running...");
            telemetry.addData("Lift target", robot.lift.getElevatorTarget());
            telemetry.addData("Lift encoder", robot.lift.getLiftEncoderPosition());
            telemetry.update();
        }
        robot.lift.setElevatorTarget(Lift.ElevatorPosition.UP);
        while (!robot.lift.actionIsCompleted() && opModeIsActive()) {
            telemetry.addData("Test", "Running...");
            telemetry.addData("Lift target", robot.lift.getElevatorTarget());
            telemetry.addData("Lift encoder", robot.lift.getLiftEncoderPosition());
            telemetry.update();
        }
        robot.lift.setElevatorTarget(Lift.ElevatorPosition.DOWN);
        while (!robot.lift.actionIsCompleted() && opModeIsActive()) {
            telemetry.addData("Test", "Running...");
            telemetry.addData("Lift target", robot.lift.getElevatorTarget());
            telemetry.addData("Lift encoder", robot.lift.getLiftEncoderPosition());
            telemetry.update();
        }
    }
}