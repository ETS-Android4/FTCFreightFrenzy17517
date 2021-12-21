package org.firstinspires.ftc.teamcode.opmodes.test;

import org.firstinspires.ftc.teamcode.misc.PositionOnField;
import org.firstinspires.ftc.teamcode.opmodes.BaseOpMode;
import org.firstinspires.ftc.teamcode.robot.LedStrip;

public class LedStripTest extends BaseOpMode {

    @Override
    public void startLoop() {
        telemetry.addData("Test", "Robot will shine after Start button has been pressed.");
        telemetry.update();
    }

    @Override
    public void main() {
        while(opModeIsActive()) {
            robot.ledStrip.setMode(LedStrip.LedStripMode.BLINK);
            robot.timer.delay(12);
            while (!robot.timer.actionIsCompleted() && opModeIsActive()) {
                telemetry.addData("Test", "Running...");
                telemetry.update();
            }
            robot.timer.delay(12);
            robot.ledStrip.setMode(LedStrip.LedStripMode.BREATHE);
            while (!robot.timer.actionIsCompleted() && opModeIsActive()) {
                telemetry.addData("Test", "Running...");
                telemetry.update();
            }
        }
    }
}
