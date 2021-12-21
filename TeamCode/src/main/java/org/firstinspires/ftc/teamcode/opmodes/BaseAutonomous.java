package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.util.ElapsedTime;

public abstract class BaseAutonomous extends BaseOpMode {


    @Override
    public void startLoop() {
        telemetry.addData("Status", "Ready to start");
        telemetry.update();
    }

    public final void execute(Runnable[] runnables) {
        execute(runnables, 10);
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
