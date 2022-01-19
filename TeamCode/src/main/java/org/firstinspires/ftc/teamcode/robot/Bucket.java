package org.firstinspires.ftc.teamcode.robot;

import static org.firstinspires.ftc.teamcode.VariablesDashboard.Elevator.bucketServoDelay;
import static org.firstinspires.ftc.teamcode.VariablesDashboard.ManipulatorConfig.positionServoDown;
import static org.firstinspires.ftc.teamcode.VariablesDashboard.ManipulatorConfig.positionServoUp;
import static org.firstinspires.ftc.teamcode.VariablesDashboard.ManipulatorConfig.positonServoForElevator;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.VariablesDashboard;

public class Bucket implements RobotModule {

    public enum BucketPosition {
        COLLECT, EJECT
    }

    private Servo servoElevator = null;


    public DistanceSensor distance = null;

    public double moveServo = positionServoDown;
    public ElapsedTime servoTimer = new ElapsedTime();

    private BucketPosition bucketPosition = BucketPosition.COLLECT;

    public BucketPosition getBucketPosition() {
        return bucketPosition;
    }

    public void setBucketPosition(BucketPosition bucketPosition) {
        if (this.bucketPosition != bucketPosition)
            servoTimer.reset();
        this.bucketPosition = bucketPosition;
    }

    private boolean freightDetected = false;

    public boolean isFreightDetected() {
        return freightDetected;
    }

    private WoENRobot robot = null;

    public Bucket(WoENRobot robot) {
        this.robot = robot;
    }

    @Override
    public void initialize() {
        distance = robot.getLinearOpMode().hardwareMap.get(DistanceSensor.class, "distance");
        servoElevator = robot.getLinearOpMode().hardwareMap.get(Servo.class, "UpDown");
    }

    public boolean actionIsCompleted() {
        return servoTimer.seconds() > bucketServoDelay;
    }

    @Override
    public void update() {
        freightDetected = distance.getDistance(DistanceUnit.CM) < 8;
        switch (bucketPosition) {
            case COLLECT:
                servoElevator.setPosition(robot.lift.getElevatorTarget() != Lift.ElevatorPosition.DOWN ?
                        positionServoUp : positonServoForElevator);
                break;
            case EJECT:
                servoElevator.setPosition(positionServoDown);
                break;
        }
    }
}
