package org.firstinspires.ftc.teamcode.robot;

import static org.firstinspires.ftc.teamcode.robot.Bucket.BucketConfig.bucketServoDelay;
import static org.firstinspires.ftc.teamcode.robot.Bucket.BucketConfig.positionServoDown;
import static org.firstinspires.ftc.teamcode.robot.Bucket.BucketConfig.positionServoUp;
import static org.firstinspires.ftc.teamcode.robot.Bucket.BucketConfig.positonServoForElevator;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.misc.TimedSensorQuery;

public class Bucket implements RobotModule {

    public DistanceSensor distance = null;
    private final TimedSensorQuery timedDistanceSensorQuery =
            new TimedSensorQuery(() -> distance.getDistance(DistanceUnit.CM), 10);
    public double moveServo = positionServoDown;
    public ElapsedTime servoTimer = new ElapsedTime();
    private Servo servoElevator = null;
    private BucketPosition bucketPosition = BucketPosition.COLLECT;
    private boolean freightDetected = false;
    private WoENRobot robot;

    public Bucket(WoENRobot robot) {
        this.robot = robot;
    }

    public BucketPosition getBucketPosition() {
        return servoTimer.seconds() > bucketServoDelay / 2 ? bucketPosition :
                (bucketPosition == BucketPosition.COLLECT ? BucketPosition.EJECT :
                        BucketPosition.COLLECT);
    }

    public void setBucketPosition(BucketPosition bucketPosition) {
        if (this.bucketPosition != bucketPosition) servoTimer.reset();
        this.bucketPosition = bucketPosition;
    }

    public boolean isFreightDetected() {
        return freightDetected;
    }

    @Override
    public void initialize() {
        distance = robot.getLinearOpMode().hardwareMap.get(DistanceSensor.class, "distance");
        servoElevator = robot.getLinearOpMode().hardwareMap.get(Servo.class, "UpDown");
    }

    public boolean actionIsCompleted() {
        return getBucketTimerStatus() &&
                (bucketPosition == BucketPosition.COLLECT || !getFreightDetectionStatus());
    }

    private boolean getBucketTimerStatus() {
        return servoTimer.seconds() > bucketServoDelay;
    }

    private boolean getFreightDetectionStatus() {
        return timedDistanceSensorQuery.getValue() < 9.5;
    }

    @Override
    public void update() {
        freightDetected = getFreightDetectionStatus();
        switch (bucketPosition) {
            case COLLECT:
                servoElevator.setPosition(
                        robot.lift.getElevatorPosition() != Lift.ElevatorPosition.DOWN ?
                                positionServoUp : positonServoForElevator);
                break;
            case EJECT:
                servoElevator.setPosition(positionServoDown);
                break;
        }
    }

    public enum BucketPosition {
        COLLECT, EJECT
    }

    @Config
    public static class BucketConfig {
        public static double positionServoUp = 0.38;
        public static double positionServoDown = 0.8;
        public static double positonServoForElevator = 0.4;
        public static double bucketServoDelay = 0.5;
    }
}
