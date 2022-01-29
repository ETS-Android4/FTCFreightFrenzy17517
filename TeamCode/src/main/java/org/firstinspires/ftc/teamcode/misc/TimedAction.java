package org.firstinspires.ftc.teamcode.misc;

import com.qualcomm.robotcore.util.ElapsedTime;

public class TimedAction {

    private Runnable action = null;
    private final double updateTimeSeconds;
    private final ElapsedTime actionTimer = new ElapsedTime();

    public TimedAction(Runnable action, double refreshRateHz) {
        this.action = action;
        updateTimeSeconds = 1 / refreshRateHz;
    }

    public TimedAction(Runnable action) {
        this(action, 50);
    }

    public void tryAction() {
        if (actionTimer.seconds() > updateTimeSeconds) {
            action.run();
            actionTimer.reset();
        }
    }
}
