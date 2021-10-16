package org.firstinspires.ftc.teamcode.opmodes;

public class ButtonSwitch {
    public boolean lastButtonState;
    private boolean activate = false;

    public boolean getState(boolean buttonState) {
        if ((buttonState) && (buttonState != lastButtonState)) {
            activate = !activate;
        }
        lastButtonState = buttonState;
        return activate;
    }
}
