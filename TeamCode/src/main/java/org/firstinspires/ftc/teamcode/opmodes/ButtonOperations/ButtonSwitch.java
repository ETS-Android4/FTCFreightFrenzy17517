package org.firstinspires.ftc.teamcode.opmodes.ButtonOperations;

public class ButtonSwitch {
    public boolean lastButtonState;
    private boolean activate = false;

    public boolean getState(boolean buttonState) {
        if (buttonState && !lastButtonState) {
            activate = !activate;
        }
        lastButtonState = buttonState;
        return activate;
    }
}
