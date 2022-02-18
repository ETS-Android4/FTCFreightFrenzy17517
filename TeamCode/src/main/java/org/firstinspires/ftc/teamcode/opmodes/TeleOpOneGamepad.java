package org.firstinspires.ftc.teamcode.opmodes;


import static org.firstinspires.ftc.teamcode.opmodes.TeleOpOneGamepad.TeleOpConfig.robotSpeedMultiplier;
import static org.firstinspires.ftc.teamcode.robot.Lift.ElevatorPosition.UP;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.misc.ButtonActivatedModes.ButtonActivated;
import org.firstinspires.ftc.teamcode.misc.ButtonOperations.ButtonSwitch;
import org.firstinspires.ftc.teamcode.misc.ButtonOperations.SmartButtonSwitch;
import org.firstinspires.ftc.teamcode.robot.Bucket;
import org.firstinspires.ftc.teamcode.robot.GyroAuto;
import org.firstinspires.ftc.teamcode.robot.LedStrip;
import org.firstinspires.ftc.teamcode.robot.Lift;

import java.util.concurrent.TimeUnit;

@TeleOp
public class TeleOpOneGamepad extends BaseOpMode {
    private final SmartButtonSwitch duck_function = new SmartButtonSwitch(() -> gamepad1.circle, (Boolean duckb) -> robot.duck.duckSpin(duckb));
    private final SmartButtonSwitch servo_elevator_function = new SmartButtonSwitch(() -> gamepad1.square, (Boolean elev) -> robot.bucket.setBucketPosition(elev ? Bucket.BucketPosition.EJECT : Bucket.BucketPosition.COLLECT));
    private final SmartButtonSwitch intake_function = new SmartButtonSwitch(() -> gamepad1.triangle, (Boolean intake) -> robot.brush.enableIntake(intake));
    private final ButtonSwitch intakeSwitch = new ButtonSwitch();
    private final ButtonSwitch speedSwitch = new ButtonSwitch();


    private ElapsedTime gyro_control = new ElapsedTime();
    public ButtonActivated BA;
    public boolean cube_bool_1 = false;
    public boolean cube_bool_2 = false;
    public double pl = 0.0;
    public int gyro_counter = 0;

    @Override
    public void main() {
        robot.init();
        gyro_control.reset();
        robot.duck.setTeleOpMode(true);
        // LED
        robot.ledStrip.setMode(LedStrip.LedStripMode.DRIVER_INDICATOR);


        while (opModeIsActive()) {
            // Movement
            //robot.movement.teleometryEncoder();
            robot.movement.setMotorPowers(
                    -gamepad1.left_stick_y * get_speed(),
                    gamepad1.right_stick_x * get_speed());
            // Switch functions
            duck_function.activate();
            servo_elevator_function.activate();
            // Others
            robot.brush.enableIntake(intakeSwitch.getState(gamepad1.triangle));
            robot.update();
            gyro_system();
            lift_function();
        }
    }

    private void gyro_system() {
        cube_bool_1 = robot.gyroAuto.gyro_status && robot.bucket.isFreightDetected();
        cube_bool_2 = robot.gyroAuto.gyro_status && !robot.bucket.isFreightDetected();
    }

    private double get_speed() {
        if (speedSwitch.getState(gamepad1.right_bumper)) pl = 1;
        else pl = 0.5 * robot.accumulator.getkVoltage();
        return pl * robotSpeedMultiplier;
    }

    private void lift_function() {
        if (gamepad1.dpad_up || cube_bool_1) {
            robot.lift.setElevatorTarget(UP);
            cube_bool_1 = robot.gyroAuto.gyro_status = false;
        }
        if (gamepad1.dpad_left) {
            robot.lift.setElevatorTarget(Lift.ElevatorPosition.MIDDLE);
        }
        if (gamepad1.dpad_down || cube_bool_2) {
            robot.lift.setElevatorTarget(Lift.ElevatorPosition.DOWN);
            cube_bool_2 = robot.gyroAuto.gyro_status = false;
        }
        if ((cube_bool_1 || cube_bool_2) && gyro_control.time(TimeUnit.SECONDS) > 0.1) {
            gyro_control.reset();
        }
    }

    @Config
    public static class TeleOpConfig {
        public static double robotSpeedMultiplier = 1.0;
    }
}
