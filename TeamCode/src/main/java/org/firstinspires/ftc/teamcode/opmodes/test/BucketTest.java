package org.firstinspires.ftc.teamcode.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opmodes.BaseOpMode;
import org.firstinspires.ftc.teamcode.robot.Bucket;

@Autonomous
public class BucketTest extends BaseOpMode {

    @Override
    public void startLoop() {
        robot.telemetryNode.getTelemetry().addData("Test", "Bucket will stay still and eject 5 seconds after Start button has been pressed.");
        robot.telemetryNode.getTelemetry().update();
    }

    @Override
    public void main() {
        robot.bucket.setBucketPosition(Bucket.BucketPosition.COLLECT);
        robot.timer.delay(5);
        while (!robot.timer.actionIsCompleted() && opModeIsActive()) {
            robot.telemetryNode.getTelemetry().addData("Test", "Running...");
            robot.telemetryNode.getTelemetry().addData("Time before ejection", robot.timer.getTimeLeft());
            robot.telemetryNode.getTelemetry().addData("Freight detected", robot.bucket.isFreightDetected());
            robot.update();
        }
        robot.bucket.setBucketPosition(Bucket.BucketPosition.EJECT);
        while (!robot.bucket.actionIsCompleted() && opModeIsActive()) {
            robot.telemetryNode.getTelemetry().addData("Test", "Running...");
            robot.telemetryNode.getTelemetry().addData("Freight detected", robot.bucket.isFreightDetected());
        }
        robot.bucket.setBucketPosition(Bucket.BucketPosition.COLLECT);
        while (!robot.bucket.actionIsCompleted() && opModeIsActive()) {
            robot.telemetryNode.getTelemetry().addData("Test", "Running...");
            robot.telemetryNode.getTelemetry().addData("Freight detected", robot.bucket.isFreightDetected());
            robot.update();
        }
    }
}
