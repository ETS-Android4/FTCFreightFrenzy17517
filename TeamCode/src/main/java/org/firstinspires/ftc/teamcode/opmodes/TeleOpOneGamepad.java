package org.firstinspires.ftc.teamcode.opmodes;


import static org.firstinspires.ftc.teamcode.opmodes.TeleOpOneGamepad.TeleOpConfig.drivetrainSpeedMultiplierDashboard;
import static org.firstinspires.ftc.teamcode.robot.Lift.ElevatorPosition.UP;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.misc.ButtonOperations.ButtonSwitch;
import org.firstinspires.ftc.teamcode.misc.ButtonOperations.SinglePressButton;
import org.firstinspires.ftc.teamcode.misc.ButtonOperations.SmartButtonSwitch;
import org.firstinspires.ftc.teamcode.robot.Bucket;
import org.firstinspires.ftc.teamcode.robot.LedStrip;
import org.firstinspires.ftc.teamcode.robot.Lift;

@TeleOp
public class TeleOpOneGamepad extends BaseOpMode {
    private final ButtonSwitch speedSwitch = new ButtonSwitch();
    private final SinglePressButton freightDetectionTrigger = new SinglePressButton();
    public boolean robotWentToShippingHub = false;
    public boolean robotWentToFreightZone = false;
    public double drivetrainSpeedMultiplier = 1.0;

    @Override
    public void main() {
        SmartButtonSwitch duck_function = new SmartButtonSwitch(() -> gamepad1.circle, robot.duck::duckSpin);
        SmartButtonSwitch servo_elevator_function =
                new SmartButtonSwitch(() -> gamepad1.square, (Boolean elev) -> robot.bucket
                        .setBucketPosition(elev ? Bucket.BucketPosition.EJECT : Bucket.BucketPosition.COLLECT));
        SmartButtonSwitch intake_function =
                new SmartButtonSwitch(() -> gamepad1.triangle, robot.brush::setEnableIntake);
        robot.duck.setTeleOpMode(true);

        while (opModeIsActive()) {
            //Driver Feedback
            freightDetectionGamepadRumble();
            // LED
            if (gamepad1.left_trigger > 0.5) robot.ledStrip.setMode(LedStrip.LedStripMode.STATIC_COLOR2);
            else if (gamepad1.right_trigger > 0.5) robot.ledStrip.setMode(LedStrip.LedStripMode.STATIC_COLOR1);
            else robot.ledStrip.setMode(LedStrip.LedStripMode.DRIVER_INDICATOR);
            // Movement
            robot.movement.setMotorPowers(-gamepad1.left_stick_y * get_speed(), gamepad1.right_stick_x * get_speed());
            // Switch functions
            duck_function.activate();
            servo_elevator_function.activate();
            // Others
            intake_function.activate();
            gyro_system();
            lift_function();
            //Update robot runctions
            robot.update();
        }
    }

    private void freightDetectionGamepadRumble() {
        if (freightDetectionTrigger.getState(robot.bucket.isFreightDetected())) gamepad1.rumble(.5, .5, 200);
    }

    private void gyro_system() {
        robotWentToShippingHub = robot.gyroAuto.isGyroTriggered() && robot.bucket.isFreightDetected();
        robotWentToFreightZone = robot.gyroAuto.isGyroTriggered() && !robot.bucket.isFreightDetected();
    }

    private double get_speed() {
        if (speedSwitch.getState(gamepad1.right_bumper)) drivetrainSpeedMultiplier = 1;
        else drivetrainSpeedMultiplier = 0.5 * robot.accumulator.getkVoltage();
        return drivetrainSpeedMultiplier * drivetrainSpeedMultiplierDashboard;
    }

    private void lift_function() {
        if (gamepad1.dpad_down || robotWentToFreightZone) {
            robot.lift.setElevatorTarget(Lift.ElevatorPosition.DOWN);
            robot.gyroAuto.resetGyroTrigger();
        }
        if (gamepad1.dpad_left) {
            robot.lift.setElevatorTarget(Lift.ElevatorPosition.MIDDLE);
            robot.gyroAuto.resetGyroTrigger(); //?
        }
        if (gamepad1.dpad_up || robotWentToShippingHub) {
            robot.lift.setElevatorTarget(UP);
            robot.gyroAuto.resetGyroTrigger();
        }
    }

    @Config
    public static class TeleOpConfig {
        public static double drivetrainSpeedMultiplierDashboard = 1.0;
    }
}
