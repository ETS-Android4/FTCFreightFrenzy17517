package org.firstinspires.ftc.teamcode.opmodes.ButtonOperations;

import java.util.function.BooleanSupplier;
import java.util.function.Consumer;

public class BS_1 {
    public boolean lastButtonState;
    public boolean buttonState;
    private boolean activate = false;
    private ButtonSwitch buttonSwitch = new ButtonSwitch();

    BooleanSupplier supplier;

    Consumer<Boolean> consumer;

    public BS_1(BooleanSupplier supp, Consumer<Boolean> cons){
        this.consumer = cons;
        this.supplier  = supp;
    }



    public void activate(){
        boolean bool = supplier.getAsBoolean();
        consumer.accept(buttonSwitch.getState(bool));
    }
}