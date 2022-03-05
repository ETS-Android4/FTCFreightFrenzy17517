package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.teamcode.robot.Duck.DuckConfig.directionDuck;

import org.firstinspires.ftc.teamcode.misc.FreightPosition;
import org.firstinspires.ftc.teamcode.robot.LedStrip;

public abstract class BaseDetectionAutonomous extends BaseAutonomous {

    protected Runnable[] upPosition() {
        return new Runnable[0];
    }

    protected Runnable[] middlePosition() {
        return new Runnable[0];
    }

    protected Runnable[] downPosition() {
        return new Runnable[0];
    }

    @Override
    public void startLoop() {
        super.startLoop();
        FreightPosition freightPosition = robot.arucoDetect.getPosition();
        robot.telemetryNode.getTelemetry().addData("Aruco position", freightPosition);
        switch (freightPosition) {
            case LEFT:
                robot.ledStrip.setMode(LedStrip.LedStripMode.BREATHE_COLOR1);
                break;
            case CENTER:
                robot.ledStrip.setMode(LedStrip.LedStripMode.BREATHE_MAGIC);
                break;
            case RIGHT:
                robot.ledStrip.setMode(LedStrip.LedStripMode.BREATHE_COLOR2);
                break;
            default:
                robot.ledStrip.setMode(LedStrip.LedStripMode.OFF);
                break;
        }
        robot.ledStrip.update();
    }

    @Override
    public void main() {
        robot.ledStrip.setMode(LedStrip.LedStripMode.DRIVER_INDICATOR);
        if (directionDuck == -1) {
            switch (robot.arucoDetect.stopCamera()) {
                case LEFT:
                    execute(downPosition());
                    break;
                case CENTER:
                    execute(middlePosition());
                    break;
                case RIGHT:
                case UNKNOWN:
                    execute(upPosition());
                    break;
            }
        } else {
            switch (robot.arucoDetect.stopCamera()) {
                case RIGHT:
                    execute(downPosition());
                    break;
                case CENTER:
                    execute(middlePosition());
                    break;
                case LEFT:
                case UNKNOWN:
                    execute(upPosition());
                    break;
            }
        }

    }

    @Override
    public void runOpMode() {
        robot.arucoDetect.initialize();
        super.runOpMode();
        //robot.arucoDetect.stopCamera();
    }
}
