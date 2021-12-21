package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.teamcode.VariablesDashboard.DuckConfig.directionDuck;

import com.qualcomm.hardware.lynx.LynxVoltageSensor;

import org.firstinspires.ftc.teamcode.misc.FreightPosition;
import org.firstinspires.ftc.teamcode.robot.LedStrip;

public abstract class BaseAutonomous extends BaseOpMode {

    protected abstract Runnable[] getUpPosition();

    protected abstract Runnable[] getMiddlePosition();

    protected abstract Runnable[] getDownPosition();

    @Override
    public void startLoop() {
        telemetry.addData("Aruco position", robot.arucoDetect.getPosition());
        telemetry.update();
        if (robot.arucoDetect.getPosition() == FreightPosition.UNKNOWN)
            robot.ledStrip.setMode(LedStrip.LedStripMode.BLINK);
        else
            robot.ledStrip.setMode(LedStrip.LedStripMode.BREATHE);
        robot.ledStrip.update();
    }

    @Override
    public void main() {
        Runnable[] actionsQueue = {};
        if (directionDuck == -1) {
            switch (robot.arucoDetect.stopCamera()) {
                case LEFT:
                    actionsQueue = getDownPosition();
                    break;
                case CENTER:
                    actionsQueue = getMiddlePosition();
                    break;
                case RIGHT:
                case UNKNOWN:
                    actionsQueue = getUpPosition();
                    break;
            }
        } else {
            switch (robot.arucoDetect.stopCamera()) {
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
        }
        for (Runnable action : actionsQueue) {
            if (opModeIsActive()) action.run();
            do
                robot.update();
            while (!robot.allActionsAreCompleted() && opModeIsActive());
        }
    }

    @Override
    public void runOpMode() {
        robot.arucoDetect.initialize();
        robot.ledStrip.resetTimer();
        super.runOpMode();
    }
}
