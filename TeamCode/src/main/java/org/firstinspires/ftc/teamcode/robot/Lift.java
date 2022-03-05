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
    private double encoderTarget = 0;
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
        int encoderPosition = getLiftEncoderPosition();
        return abs(encoderPosition - middleTargetElevator) < abs(encoderPosition - upTargetElevator) ?
                ElevatorPosition.MIDDLE : ElevatorPosition.UP;
    }

    public void setElevatorTarget(ElevatorPosition elevatorPosition) {
        this.elevatorTarget = elevatorPosition;

        switch (elevatorPosition) {
            case DOWN:
                encoderTarget = downTargetElevator;
                break;
            case MIDDLE:
                encoderTarget = middleTargetElevator;
                break;
            case UP:
                encoderTarget = upTargetElevator;
                break;
        }

        queuebool = false;
    }

    public int getLiftEncoderPosition() {
        return motorLift.getCurrentPosition() - liftEncoderOffset;
    }

    public void update() {
        double motorSpeed;
        if (elevatorTarget == ElevatorPosition.DOWN) {
            if (!limitSwitch.getState()) {
                double error = encoderTarget - getLiftEncoderPosition();
                motorSpeed = (((error < 0 ? error * kP : 0) - homingPower) * robot.accumulator.getkVoltage());
                queuebool = false;
            } else {
                motorSpeed = 0;
                if (!queuebool) liftEncoderOffset = motorLift.getCurrentPosition();
                queuebool = true;
            }
        } else {
            double error = encoderTarget - getLiftEncoderPosition();
            if (abs(error) > errorThreshold) {
                motorSpeed = (error * kP * robot.accumulator.getkVoltage());
                queuebool = false;
            } else {
                motorSpeed = 0;
                queuebool = true;
            }
        }
        //liftCommandSender.send(motorSpeed);
        liftAccelerationLimiter.setSpeed(motorSpeed);
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
        public static double downTargetElevator = 0;
        public static double middleTargetElevator = 650;
        public static double upTargetElevator = 1340;
    }
}
