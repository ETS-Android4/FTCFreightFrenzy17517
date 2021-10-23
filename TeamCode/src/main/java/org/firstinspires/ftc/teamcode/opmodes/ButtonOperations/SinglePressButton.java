package org.firstinspires.ftc.teamcode.opmodes.ButtonOperations;

public class SinglePressButton {
    public boolean lastButtonState;
    private boolean activate = false;

    public boolean getState(boolean buttonState) {
        boolean result = buttonState && !lastButtonState;
        lastButtonState = buttonState;
        return result;
    }
}
