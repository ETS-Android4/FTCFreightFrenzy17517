package org.firstinspires.ftc.teamcode.misc.ButtonOperations;

import java.util.function.BooleanSupplier;
import java.util.function.Consumer;

public class SmartButtonSwitch {
    public boolean lastButtonState;
    public boolean buttonState;
    private boolean activate = false;
    private ButtonSwitch buttonSwitch = new ButtonSwitch();
    BooleanSupplier supplier;
    Consumer<Boolean> consumer;

    public SmartButtonSwitch(BooleanSupplier supp, Consumer<Boolean> cons) {
        this.consumer = cons;
        this.supplier = supp;
    }

    public void activate() {
        boolean bool = supplier.getAsBoolean();
        consumer.accept(buttonSwitch.getState(bool));
    }
}