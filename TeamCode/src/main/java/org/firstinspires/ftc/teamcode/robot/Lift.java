package org.firstinspires.ftc.teamcode.robot;

import static org.firstinspires.ftc.teamcode.robot.Lift.LiftConfig.downTargetElevator;
import static org.firstinspires.ftc.teamcode.robot.Lift.LiftConfig.errorThreshold;
import static org.firstinspires.ftc.teamcode.robot.Lift.LiftConfig.homingPower;
import static org.firstinspires.ftc.teamcode.robot.Lift.LiftConfig.kP;
import static org.firstinspires.ftc.teamcode.robot.Lift.LiftConfig.maxAcceleration;
import static org.firstinspires.ftc.teamcode.robot.Lift.LiftConfig.middleTargetElevator;
import static org.firstinspires.ftc.teamcode.robot.Lift.LiftConfig.upTargetElevator;
import static java.lang.Math.abs;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.teamcode.misc.CommandSender;
import org.firstinspires.ftc.teamcode.misc.MotorAccelerationLimiter;

public class Lift implements RobotModule {

    private final WoENRobot robot;
    public boolean queuebool = true;
    public DigitalChannel limitSwitch = null;
    private DcMotorEx motorLift = null;
    private final CommandSender liftCommandSender = new CommandSender((double value) -> motorLift.setPower(value));
    private final MotorAccelerationLimiter liftAccelerationLimiter = new MotorAccelerationLimiter(liftCommandSender::send,maxAcceleration);
    private ElevatorPosition elevatorTarget = ElevatorPosition.DOWN;
    private int liftEncoderOffset = 0;

    public Lift(WoENRobot robot) {
        this.robot = robot;
    }

    public void initialize() {
        limitSwitch = robot.getLinearOpMode().hardwareMap.get(DigitalChannel.class, "limitSwitch");
        limitSwitch.setMode(DigitalChannel.Mode.INPUT);
        motorLift = robot.getLinearOpMode().hardwareMap.get(DcMotorEx.class, "E1");
        motorLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorLift.setDirection(DcMotorEx.Direction.REVERSE);
    }

    public boolean actionIsCompleted() {
        return queuebool;
    }

    public ElevatorPosition getElevatorPosition() {
        if (elevatorTarget == ElevatorPosition.DOWN && queuebool) return ElevatorPosition.DOWN;
        int encoderPosition = getOffsetEncoderPosition();
        return abs(encoderPosition - middleTargetElevator) < abs(encoderPosition - upTargetElevator) ?
                ElevatorPosition.MIDDLE : ElevatorPosition.UP;
    }

    public void setElevatorTarget(ElevatorPosition elevatorTarget) {
        this.elevatorTarget = elevatorTarget;
    }

    public int getEncoderTarget() {
        switch (elevatorTarget) {
            case UP:
                return upTargetElevator;
            case MIDDLE:
                return middleTargetElevator;
            case DOWN:
            default:
                return downTargetElevator;
        }
    }

    public int getOffsetEncoderPosition() {
        return motorLift.getCurrentPosition() - liftEncoderOffset;
    }

    public void update() {
        int error = getEncoderTarget() - getOffsetEncoderPosition();
        double power;
        if (elevatorTarget == ElevatorPosition.DOWN) {
            if (!limitSwitch.getState()) power = ((error < 0 ? error * kP : 0) - homingPower);
            else {
                power = 0;
                if (!queuebool) liftEncoderOffset = motorLift.getCurrentPosition();
            }
        }
        else power = abs(error) > errorThreshold ? error * kP : 0;

        queuebool = power == 0;
        //liftCommandSender.send(power * robot.accumulator.getkVoltage());
        liftAccelerationLimiter.setSpeed(power * robot.battery.getkVoltage());

}

    public enum ElevatorPosition {
        DOWN, MIDDLE, UP
    }

    @Config
    public static class LiftConfig {
        public static double homingPower = 0.4;
        public static double kP = 0.01;
        public static double maxAcceleration = 10;
        public static int errorThreshold = 10;
        public static int downTargetElevator = 0;
        public static int middleTargetElevator = 650;
        public static int upTargetElevator = 1340;
    }
}
