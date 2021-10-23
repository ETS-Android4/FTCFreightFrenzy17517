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
public class Autonom_normal extends LinearOpMode {
    public DcMotor R_1 = null;
    public DcMotor L_1 = null;
    public double x = 0.0;
    public double y = 0.0;
    public double OX = 0.0;
    public double OY = 0.0;
    public double L = 100;

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
        rotation(-1,90,1.0);


    }

    public void rotation(double direction, double angle, double speed) {
        double m1 = L_1.getCurrentPosition();
        double m2 = R_1.getCurrentPosition();
        angle = angle/360;
        while(direction*angle/360*((L_1.getCurrentPosition() - m1) - (R_1.getCurrentPosition() - m2))/L > direction*(m1 - m2)/L){
            R_1.setPower(Range.clip((speed * ((angle * angle) / angle)) * VariablesDashboard.TeleOpConfig.robotSpeed, -1.0, 1.0));
            L_1.setPower(Range.clip((-speed * ((angle * angle) / angle)) * VariablesDashboard.TeleOpConfig.robotSpeed, -1.0, 1.0));
        }
        R_1.setPower(Range.clip((0) * VariablesDashboard.TeleOpConfig.robotSpeed, -1.0, 1.0));
        L_1.setPower(Range.clip((0) * VariablesDashboard.TeleOpConfig.robotSpeed, -1.0, 1.0));
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
