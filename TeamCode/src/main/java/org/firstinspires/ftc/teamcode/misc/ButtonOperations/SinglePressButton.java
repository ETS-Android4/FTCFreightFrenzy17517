package org.firstinspires.ftc.teamcode.misc.ButtonOperations;

public class SinglePressButton {
    public boolean lastButtonState;
    private final boolean activate = false;

    public boolean getState(boolean buttonState) {
        boolean result = buttonState && !lastButtonState;
        lastButtonState = buttonState;
        return result;
    }

    public boolean getStateInt(boolean buttonState, int variants) {
        boolean result = buttonState && !lastButtonState;
        lastButtonState = buttonState;
        return result;
    }
}
