package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.teamcode.opencv.ArucoDetect.centreOfDuck;
import static org.firstinspires.ftc.teamcode.opencv.ArucoDetect.timePosition;

import com.qualcomm.robotcore.util.ElapsedTime;

public abstract class BaseAutonomous extends BaseOpMode {


    @Override
    public void startLoop() {
        robot.telemetryNode.getTelemetry().addData("Status", "Ready to start");
        robot.telemetryNode.getTelemetry().addData("offset", centreOfDuck);
        robot.telemetryNode.getTelemetry().addData("timePosition", timePosition);
        robot.telemetryNode.getTelemetry().update();
    }

    public final void execute(Runnable[] runnables) {
        execute(runnables, 4);
    }

    public final void execute(Runnable[] runnables, double timeoutSeconds) {
        for (Runnable action : runnables) {
            ElapsedTime elapsedTime = new ElapsedTime();
            if (opModeIsActive()) action.run();
            do
                robot.update();
            while (!robot.allActionsAreCompleted() && opModeIsActive() && elapsedTime.seconds() < timeoutSeconds);
        }
    }

}
