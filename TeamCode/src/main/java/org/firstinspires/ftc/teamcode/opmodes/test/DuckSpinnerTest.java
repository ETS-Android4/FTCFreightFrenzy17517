package org.firstinspires.ftc.teamcode.opmodes.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.misc.AllianceColor;
import org.firstinspires.ftc.teamcode.misc.StartingPosition;
import org.firstinspires.ftc.teamcode.opmodes.BaseOpMode;

@Autonomous
public class DuckSpinnerTest extends BaseOpMode {

    @Override
    public void startLoop() {
        robot.telemetryNode.getTelemetry()
                .addData("Test", "Duck spinner will spin for N seconds after Start button has been pressed.");
        robot.telemetryNode.getTelemetry().update();
    }

    @Override
    public void main() {
        robot.duck.setFieldPosition(AllianceColor.RED, StartingPosition.LEFT);
        robot.duck.duckSpin(true);
        while (!robot.duck.actionIsCompleted() && opModeIsActive()) {
            robot.telemetryNode.getTelemetry().addData("Test", "Running...");
            robot.telemetryNode.getTelemetry().addData("Field position", "RED");
            robot.update();
        }
        robot.duck.setFieldPosition(AllianceColor.BLUE, StartingPosition.LEFT);
        robot.duck.duckSpin(false);
        robot.duck.duckSpin(true);
        while (!robot.duck.actionIsCompleted() && opModeIsActive()) {
            robot.telemetryNode.getTelemetry().addData("Test", "Running...");
            robot.telemetryNode.getTelemetry().addData("Field position", "BLUE");
            robot.update();
        }
    }
}
