package org.firstinspires.ftc.teamcode;
//test

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

@TeleOp
public class Move extends LinearOpMode {
    public DcMotor RightMotor = null;
    public DcMotor LeftMotor = null;
    public double x = 0.0;
    public double y = 0.0;

    @Config
    public static class RobotConfig {
        public static double speed = 1.0;
    }

    @Override
    public void runOpMode() {
        RightMotor = hardwareMap.get(DcMotor.class, "R1");
        LeftMotor = hardwareMap.get(DcMotor.class, "L1");
        RightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        LeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        FtcDashboard dashboard = FtcDashboard.getInstance();
        TelemetryPacket packet = new TelemetryPacket();

        waitForStart();
        while (opModeIsActive()) {
            y = gamepad1.right_stick_x;
            x = gamepad1.left_stick_y;
            //packet.put("x", x);
            //packet.put("y", y);
            packet.fieldOverlay()
                    .setFill("red")
                    .fillRect(-20, -20, 10, 10);
            RightMotor.setPower(Range.clip((x - y) * RobotConfig.speed, -1.0, 1.0));
            LeftMotor.setPower(Range.clip((x + y) * RobotConfig.speed, -1.0, 1.0));
            dashboard.sendTelemetryPacket(packet);
        }
    }
}
