package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.VariablesDashboard.ManipulatorConfig.*;
import static org.firstinspires.ftc.teamcode.VariablesDashboard.TeleOpConfig.robotSpeed;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.opmodes.ButtonActivatedModes.ButtonActivated;
import org.firstinspires.ftc.teamcode.opmodes.ButtonOperations.ButtonSwitch;
import org.firstinspires.ftc.teamcode.opmodes.RobotModules;
import org.firstinspires.ftc.teamcode.opmodes.Tele;

@TeleOp
public class TeleOpOneGamepad extends LinearOpMode {
    private final RobotModules robotModules = new RobotModules(this);
    // private BS_1 duck_function = new BS_1(() -> gamepad1.square,(Boolean duckb) -> duck.DuckSpin(duckb));
    ButtonSwitch manipulatorSwitch = new ButtonSwitch();
    private Tele telemetry_function = new Tele(this); //TODO is broken
    public ButtonActivated BA;

    public double x = 0.0;
    public double y = 0.0;
    public double OX = 0.0;
    public double OY = 0.0;

    @Override
    public void runOpMode() {

        robotModules.init();

        waitForStart();
        while (opModeIsActive()) {
            robotModules.movement.setMotorPowers(-gamepad1.left_stick_y * robotSpeed,
                    gamepad1.right_stick_x * robotSpeed);
            if (manipulatorSwitch.getState(gamepad1.dpad_up))
                robotModules.manipulator.MoveServo(Manipulator.ManipulatorPosition.UP);
            else
                robotModules.manipulator.MoveServo(Manipulator.ManipulatorPosition.DOWN);
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
