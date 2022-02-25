package org.firstinspires.ftc.teamcode.misc;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import java.util.function.DoubleConsumer;

public class CommandSender {

    private static final double internalResolution = 32767;
    private static final double apiResolution = 1.0;
    private final DoubleConsumer doubleConsumer;
    private final ElapsedTime sendTimer = new ElapsedTime();
    private int lastSentInternalValue = Integer.MIN_VALUE;
    private double sendTimeoutSeconds = 1;

    public CommandSender(DoubleConsumer doubleConsumer) {
        this.doubleConsumer = doubleConsumer;
    }

    public CommandSender(DoubleConsumer doubleConsumer, double sendTimeoutSeconds) {
        this(doubleConsumer);
        this.sendTimeoutSeconds = sendTimeoutSeconds;
    }

    private int apiToInternal(double value) {
        return (int) Range.scale(value, -apiResolution, apiResolution, -internalResolution, internalResolution);
    }

    public void send(double value) {
        if (sendTimer.seconds() > sendTimeoutSeconds || apiToInternal(value) != lastSentInternalValue) {
            doubleConsumer.accept(value);
            lastSentInternalValue = apiToInternal(value);
        }
    }


}
