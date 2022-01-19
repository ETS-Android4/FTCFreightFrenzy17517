package org.firstinspires.ftc.teamcode.misc;

import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.function.DoubleSupplier;

public class TimedSensorQuery {

    private DoubleSupplier sensorValueSupplier = null;

    private double lastValue = Double.NaN;

    private double updateTimeSeconds;

    private ElapsedTime queryTimer = new ElapsedTime();

    public TimedSensorQuery(DoubleSupplier sensorValueSupplier, double refreshRateHz) {
        this.sensorValueSupplier = sensorValueSupplier;
        updateTimeSeconds = 1 / refreshRateHz;
    }

    public TimedSensorQuery(DoubleSupplier sensorValueSupplier) {
        this(sensorValueSupplier, 50);
    }

    public double getValue() {
        if (Double.isNaN(lastValue))
            lastValue = sensorValueSupplier.getAsDouble();
        else if (queryTimer.seconds() > updateTimeSeconds) {
            lastValue = sensorValueSupplier.getAsDouble();
            queryTimer.reset();
        }
        return lastValue;
    }
}
