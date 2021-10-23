package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.opmodes.ButtonOperations.BS_1;
import org.firstinspires.ftc.teamcode.opmodes.ButtonActivatedModes.ButtonActivated;
import org.firstinspires.ftc.teamcode.opmodes.ButtonOperations.ButtonSwitch;
import org.firstinspires.ftc.teamcode.opmodes.ButtonActivatedModes.Duck;
import org.firstinspires.ftc.teamcode.opmodes.Global;
import org.firstinspires.ftc.teamcode.opmodes.Tele;

@TeleOp
public class TeleOP_1 extends LinearOpMode {
    public DcMotor R_1 = null;
    public DcMotor L_1 = null;
  //  private Duck duck = new Duck(this);
    private Manipulator manipulator = new Manipulator(this);
    private Movement move = new Movement(this);
    private ButtonSwitch duckSwitch = new ButtonSwitch();
    private Global global = new Global(this);
   // private BS_1 duck_function = new BS_1(() -> gamepad1.square,(Boolean duckb) -> duck.DuckSpin(duckb));
    private Tele telemetry_function = new Tele(this);
    public ButtonActivated BA;

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
        manipulator.initManip();
        move.init();

        waitForStart();
        while (opModeIsActive()) {
            move.Motor(gamepad1.left_stick_y, gamepad1.right_stick_x);
            if (gamepad1.dpad_up) manipulator.MoveServo(0.7);
            if (gamepad1.dpad_down) manipulator.MoveServo(0.05);
            //duck_function.activate();
            //telemetry_function.activate();
           // Drawing();
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
