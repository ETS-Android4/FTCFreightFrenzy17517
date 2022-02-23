package org.firstinspires.ftc.teamcode.opmodes.test;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.opmodes.BaseOpMode;
import org.firstinspires.ftc.teamcode.robot.LedStrip;

@Config
@TeleOp
public class LedStripTest extends BaseOpMode {

    public static LedStrip.LedStripMode ledStripMode = LedStrip.LedStripMode.BREATHE_MAGIC;

    @Override
    public void startLoop() {
        robot.telemetryNode.getTelemetry().addData("Test", "Robot will shine after Start button had been pressed.");
        robot.telemetryNode.getTelemetry().update();
    }

    @Override
    public void main() {
        robot.telemetryNode.getTelemetry().addData("Test", "Running...");
        robot.telemetryNode.getTelemetry().update();
        while (opModeIsActive()) {
            robot.ledStrip.setMode(ledStripMode);
            robot.ledStrip.update();
        }
    }
}
