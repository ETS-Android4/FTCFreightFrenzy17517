package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.teamcode.VariablesDashboard.DuckConfig.directionDuck;

import org.firstinspires.ftc.teamcode.misc.FreightPosition;
import org.firstinspires.ftc.teamcode.robot.LedStrip;

public abstract class BaseDetectionAutonomous extends BaseAutonomous {

    protected abstract Runnable[] upPosition();

    protected abstract Runnable[] middlePosition();

    protected abstract Runnable[] downPosition();

    @Override
    public void startLoop(){
        super.startLoop();
        telemetry.addData("Aruco position", robot.arucoDetect.getPosition());
        if (robot.arucoDetect.getPosition() == FreightPosition.UNKNOWN)
            robot.ledStrip.setMode(LedStrip.LedStripMode.BLINK);
        else
            robot.ledStrip.setMode(LedStrip.LedStripMode.BREATHE);
        robot.ledStrip.update();
    }

    @Override
    public void main() {
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
    }
}
