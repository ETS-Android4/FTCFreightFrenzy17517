package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.teamcode.VariablesDashboard.ManipulatorConfig.AutoTele;
import static org.firstinspires.ftc.teamcode.robot.Lift.ElevatorPosition.UP;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.misc.ButtonActivatedModes.ButtonActivated;
import org.firstinspires.ftc.teamcode.misc.ButtonOperations.ButtonSwitch;
import org.firstinspires.ftc.teamcode.misc.ButtonOperations.SmartButtonSwitch;
import org.firstinspires.ftc.teamcode.robot.Bucket;
import org.firstinspires.ftc.teamcode.robot.GyroAuto;
import org.firstinspires.ftc.teamcode.robot.LedStrip;
import org.firstinspires.ftc.teamcode.robot.Lift;

import java.lang.reflect.GenericDeclaration;

@TeleOp
public class TeleOpOneGamepad extends BaseOpMode {
    private final SmartButtonSwitch duck_function = new SmartButtonSwitch(() -> gamepad1.circle, (Boolean duckb) -> robot.duck.duckSpin(duckb));
    private final SmartButtonSwitch servo_elevator_function = new SmartButtonSwitch(() -> gamepad1.square, (Boolean elev) -> robot.bucket.setBucketPosition(elev ? Bucket.BucketPosition.EJECT : Bucket.BucketPosition.COLLECT));
    private final SmartButtonSwitch intake_function = new SmartButtonSwitch(() -> gamepad1.triangle, (Boolean intake) -> robot.brush.enableIntake(intake));
    private final ButtonSwitch intakeSwitch = new ButtonSwitch();
    private final ButtonSwitch speedSwitch = new ButtonSwitch();


    public ButtonActivated BA;
    public boolean cube_bool_1 = false;
    public boolean cube_bool_2 = false;
    public boolean t = true;
    public boolean u = true;
    public double pl = 0.0;
    private GyroAuto gyro_auto = new GyroAuto(robot);

    @Override
    public void main() {
        robot.init();
        AutoTele = true;
        robot.duck.Teleop();
        robot.ledStrip.setMode(LedStrip.LedStripMode.INDICATOR);
        gyro_auto.initialize();
        while (opModeIsActive()) {
            // Local conditions
            //if(gamepad1.left_bumper){ obnul(u); }
            robot.movement.setMotorPowers(-gamepad1.left_stick_y * get_speed(), gamepad1.right_stick_x * pl);
            // Switch functions
            duck_function.activate();
            servo_elevator_function.activate();
            // Others
            robot.brush.enableIntake(t && intakeSwitch.getState(gamepad1.triangle));
            lift_function();
            robot.update();
            gyro_auto.reaction();
            cube_bool_1 = gyro_auto.gyro_status;
        }
    }

    private double get_speed() {
        if (speedSwitch.getState(gamepad1.right_bumper)) {
            pl = 1;
        } else {
            pl = 0.5;
        }
        return pl;
    }

    /*
    private void obnul(boolean i) {
        if (i) {
            while (gamepad1.left_bumper && !robot.lift.limitSwitch.getState()) {
                robot.lift.motorLift.setPower(-1);
                u = false;
            }
            robot.lift.resetEncoderElevator();
        }
    }
     */

    private void lift_function() {
        if (gamepad1.dpad_up || cube_bool_1) {
            cube_bool_1 = false;
            gyro_auto.gyro_status = false;
            robot.lift.setElevatorTarget(UP);
            t = false;
        }
        if (gamepad1.dpad_left) {
            robot.lift.setElevatorTarget(Lift.ElevatorPosition.MIDDLE);
            t = false;
        }
        if (gamepad1.dpad_down || cube_bool_2) {
            cube_bool_2 = false;
            robot.lift.setElevatorTarget(Lift.ElevatorPosition.DOWN);
            t = true;
        }
    }

    /*public void Drawing() {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("x", x);
        packet.put("y", y);
        packet.fieldOverlay()
                .setFill("red")
                .fillRect(OX / 1000, OY / 1000, 40, 40);
        dashboard.sendTelemetryPacket(packet);
    }*/
}
