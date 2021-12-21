package org.firstinspires.ftc.teamcode.opmodes.test;

import org.firstinspires.ftc.teamcode.misc.PositionOnField;
import org.firstinspires.ftc.teamcode.opmodes.BaseOpMode;

public class DuckSpinnerTest extends BaseOpMode {

    @Override
    public void startLoop() {
        telemetry.addData("Test", "Duck spinner will spin for N seconds after Start button has been pressed.");
        telemetry.update();
    }

    @Override
    public void main() {
        robot.duck.redOrBlue(PositionOnField.RED);
        robot.duck.duckSpin(true);
        while (!robot.duck.actionIsCompleted() && opModeIsActive()) {
            telemetry.addData("Test", "Running...");
            telemetry.addData("Field position", "Red");
            telemetry.update();
        }
        robot.duck.redOrBlue(PositionOnField.BLUE);
        robot.duck.duckSpin(false);
        robot.duck.duckSpin(true);
        while (!robot.duck.actionIsCompleted() && opModeIsActive()) {
            telemetry.addData("Test", "Running...");
            telemetry.addData("Field position", "BLUE");
            telemetry.update();
        }
    }
}
