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
        telemetry.addData("Test", "Robot will shine after Start button had been pressed.");
        telemetry.update();
    }

    @Override
    public void main() {
        telemetry.addData("Test", "Running...");
        telemetry.update();
        while (opModeIsActive()) {
            robot.ledStrip.setMode(ledStripMode);
            robot.ledStrip.update();
        }
    }
}
