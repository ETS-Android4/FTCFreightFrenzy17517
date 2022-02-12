package org.firstinspires.ftc.teamcode.opmodes.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.misc.PositionOnField;
import org.firstinspires.ftc.teamcode.misc.PositionToSearch;
import org.firstinspires.ftc.teamcode.opmodes.BaseOpMode;

@Autonomous
public class DuckSpinnerTest extends BaseOpMode {

    @Override
    public void startLoop() {
        FtcDashboard.getInstance().getTelemetry().addData("Test", "Duck spinner will spin for N seconds after Start button has been pressed.");
        FtcDashboard.getInstance().getTelemetry().update();
        telemetry.addData("Test", "Duck spinner will spin for N seconds after Start button has been pressed.");
        telemetry.update();
    }

    @Override
    public void main() {
        robot.duck.redOrBlue(PositionOnField.RED, PositionToSearch.LEFT);
        robot.duck.duckSpin(true);
        while (!robot.duck.actionIsCompleted() && opModeIsActive()) {
            telemetry.addData("Test", "Running...");
            telemetry.addData("Field position", "Red");
            telemetry.update();
        }
        robot.duck.redOrBlue(PositionOnField.BLUE, PositionToSearch.LEFT);
        robot.duck.duckSpin(false);
        robot.duck.duckSpin(true);
        while (!robot.duck.actionIsCompleted() && opModeIsActive()) {
            telemetry.addData("Test", "Running...");
            telemetry.addData("Field position", "BLUE");
            robot.update();
            telemetry.update();
        }
    }
}
