package org.firstinspires.ftc.teamcode.misc.ButtonOperations;

import java.util.function.BooleanSupplier;
import java.util.function.Consumer;

public class SmartButtonSwitch {
    BooleanSupplier supplier;
    Consumer<Boolean> consumer;
    private final boolean activate = false;
    private final ButtonSwitch buttonSwitch = new ButtonSwitch();

    public SmartButtonSwitch(BooleanSupplier supp, Consumer<Boolean> cons) {
        this.consumer = cons;
        this.supplier = supp;
    }

    public void activate() {
        boolean bool = supplier.getAsBoolean();
        consumer.accept(buttonSwitch.getState(bool));
    }
}