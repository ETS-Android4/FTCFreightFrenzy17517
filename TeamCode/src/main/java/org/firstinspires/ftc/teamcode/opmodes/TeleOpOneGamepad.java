package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.teamcode.VariablesDashboard.TeleOpConfig.robotSpeed;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.misc.ButtonOperations.SmartButtonSwitch;
import org.firstinspires.ftc.teamcode.misc.ButtonActivatedModes.ButtonActivated;
import org.firstinspires.ftc.teamcode.misc.ButtonOperations.ButtonSwitch;
import org.firstinspires.ftc.teamcode.robot.Duck;
import org.firstinspires.ftc.teamcode.robot.Elevator;
import org.firstinspires.ftc.teamcode.robot.RobotModules;

@TeleOp
public class TeleOpOneGamepad extends LinearOpMode {
    private final RobotModules robotModules = new RobotModules(this);
    private Duck duck = new Duck(this);
    private ButtonSwitch buttonSwitch = new ButtonSwitch();
    private SmartButtonSwitch duck_function = new SmartButtonSwitch(() -> gamepad1.square,(Boolean duckb) -> robotModules.duck.DuckSpin(duckb));
    private SmartButtonSwitch elevator_function = new SmartButtonSwitch(() -> gamepad1.triangle,(Boolean elev) -> robotModules.elevator.MoveServoForElevator(elev));
    private SmartButtonSwitch intake_function = new SmartButtonSwitch(() -> gamepad1.circle,(Boolean intake) -> robotModules.brush.brushMotorMove(intake));
//    private Tele telemetry_function = new Tele(this);
    public ButtonActivated BA;



    public boolean u = true;
    public double x = 0.0;
    public double y = 0.0;
    public double OX = 0.0;
    public double OY = 0.0;


    @Override
    public void runOpMode() {

        robotModules.init();

        waitForStart();
        while (opModeIsActive()) {
            RobotModules.movement.setMotorPowers(-gamepad1.left_stick_y * robotSpeed, gamepad1.right_stick_x * robotSpeed);
            if(gamepad1.left_bumper){ obnul(u); }
            duck_function.activate();
            elevator_function.activate();
            intake_function.activate();
//            telemetry_function.activate();
            lift_function();
            robotModules.updateForTeleop();
            Drawing();
        }
    }



    private void obnul(boolean i){
        if(i){
            u = false;
            while (gamepad1.left_bumper) {
                robotModules.elevator.motorLift.setPower(-1);
            }
            robotModules.elevator.init();
        }
    }

    private void lift_function(){
        if(gamepad1.dpad_up){ robotModules.elevator.ElevatorPosition(Elevator.ElevatorPosition.UP); }
        if(gamepad1.dpad_left){ robotModules.elevator.ElevatorPosition(Elevator.ElevatorPosition.MIDDLE); }
        if(gamepad1.dpad_down){ robotModules.elevator.ElevatorPosition(Elevator.ElevatorPosition.DOWN); }
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