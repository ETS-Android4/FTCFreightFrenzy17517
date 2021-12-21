package org.firstinspires.ftc.teamcode.opmodes.test;

import org.firstinspires.ftc.teamcode.opmodes.BaseOpMode;

public class BrushTest extends BaseOpMode {

    @Override
    public void startLoop() {
        telemetry.addData("Test", "Brush will spin forward for 10 seconds after Start button has been pressed");
        telemetry.update();
    }

    @Override
    public void main() {
        robot.brush.enableIntake(true);
        robot.timer.delay(10);
        while (!robot.timer.actionIsCompleted() && opModeIsActive()) {
            telemetry.addData("Test", "Running...");
            telemetry.update();
        }
    }
}
