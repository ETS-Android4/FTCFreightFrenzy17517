package org.firstinspires.ftc.teamcode.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opmodes.BaseOpMode;

@Autonomous
public class BrushTest extends BaseOpMode {

    @Override
    public void startLoop() {
        robot.telemetryNode.getTelemetry()
                .addData("Test", "Brush will spin forward for 10 seconds after Start button has been pressed");
        robot.telemetryNode.getTelemetry().update();
    }

    @Override
    public void main() {
        robot.brush.setEnableIntake(true);
        robot.timer.delay(10);
        while (!robot.timer.actionIsCompleted() && opModeIsActive()) {
            robot.telemetryNode.getTelemetry().addData("Test", "Running...");
            robot.update();
        }
    }
}
