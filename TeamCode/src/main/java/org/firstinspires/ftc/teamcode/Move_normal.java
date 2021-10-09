package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

@TeleOp
public class Move_normal extends LinearOpMode {
    public DcMotor R_1 = null;
    public DcMotor L_1 = null;
    public double x = 0.0;
    public double y = 0.0;
    public double OX = 0.0;
    public double OY = 0.0;

    @Config
    public static class RobotConfig {
        public static double speed = 1.0;
    }

    @Override
    public void runOpMode() {
        R_1 = hardwareMap.get(DcMotor.class, "R1");
        L_1 = hardwareMap.get(DcMotor.class, "L1");
        R_1.setDirection(DcMotorSimple.Direction.REVERSE);
        L_1.setDirection(DcMotorSimple.Direction.FORWARD);

        waitForStart();
        while (opModeIsActive()) {
            y = gamepad1.right_stick_x;
            x = gamepad1.left_stick_y;
            R_1.setPower(Range.clip((x + y) * RobotConfig.speed, -1.0, 1.0));
            L_1.setPower(Range.clip((x - y) * RobotConfig.speed, -1.0, 1.0));
            OX = OX + x;
            OY = OY + y;
            Drawing();
        }
    }
    public void Drawing() {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("x", x);
        packet.put("y", y);
        packet.fieldOverlay()
                .setFill("red")
                .fillRect(OX / 1000, OY / 1000, 40, 40);
        dashboard.sendTelemetryPacket(packet);
    }
}
