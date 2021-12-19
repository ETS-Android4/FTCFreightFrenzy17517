package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robot.RobotModules;

public abstract class BaseAutonomous extends LinearOpMode {

    protected abstract Runnable[] getUpPosition();

    protected abstract Runnable[] getMiddlePosition();

    protected abstract Runnable[] getDownPosition();

    FtcDashboard dashboard;

    private final RobotModules robotModules = new RobotModules(this);

    private void startLoop() {
        RobotModules.brush.breatheLed();
    }

    @Override
    public void waitForStart() {
        RobotModules.brush.resetLedBreatheTimer();
        while (!isStarted() && !isStopRequested()) {
            startLoop();
        }
        super.waitForStart();
    }

    @Override
    public void runOpMode() {
        dashboard = FtcDashboard.getInstance();
        robotModules.init();
        RobotModules.arucoDetect.init(hardwareMap);
        RobotModules.brush.resetLedBreatheTimer();
        waitForStart();
        RobotModules.arucoDetect.getPosition();
        Runnable[] actionsQueue = {};
        switch (RobotModules.arucoDetect.stopCamera()) {
            case RIGHT:
                actionsQueue = getDownPosition();
                break;
            case CENTER:
                actionsQueue = getMiddlePosition();
                break;
            case LEFT:
            case UNKNOWN:
                actionsQueue = getUpPosition();
                break;
        }

        int queueIndex = 0;
        while (opModeIsActive() && queueIndex < actionsQueue.length) {
            Runnable action = actionsQueue[queueIndex];
            action.run();
            robotModules.update();
            if (robotModules.line()) {
                queueIndex++;
            }
        }

    }
}
