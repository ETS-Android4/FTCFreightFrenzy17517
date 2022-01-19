package org.firstinspires.ftc.teamcode.opmodes.test;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.misc.PositionOnField;
import org.firstinspires.ftc.teamcode.opmodes.BaseOpMode;
import org.firstinspires.ftc.teamcode.robot.LedStrip;

@Config
@TeleOp
public class LedStripTest extends BaseOpMode {

    public static LedStrip.LedStripMode ledStripMode = LedStrip.LedStripMode.BLINK;

    private final ElapsedTime telemetryTimer = new ElapsedTime();

    @Override
    public void startLoop() {
        if(telemetryTimer.seconds()>5) {
            telemetry.addData("Test", "Robot will shine after Start button had been pressed.");
            telemetry.update();
            telemetryTimer.reset();
        }
        robot.ledStrip.setMode(ledStripMode);
        robot.ledStrip.update();
    }

    @Override
    public void main() {
        telemetry.addData("Test", "Running...");
        telemetry.update();
        while (opModeIsActive()) {
            robot.ledStrip.setMode(ledStripMode);
            robot.update();
        }
    }
}
