package org.firstinspires.ftc.teamcode.misc;

import static java.lang.Math.abs;
import static java.lang.Math.min;
import static java.lang.Math.signum;

import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.function.DoubleConsumer;

public class MotorAccelerationLimiter {

    private final DoubleConsumer doubleConsumer;

    public void setMaxAcceleration(double maxAcceleration) {
        this.maxAcceleration = maxAcceleration;
    }

    private double maxAcceleration = 0.1;

    public MotorAccelerationLimiter(DoubleConsumer doubleConsumer){
        this.doubleConsumer = doubleConsumer;
        loopTime.reset();
    }

    public MotorAccelerationLimiter(DoubleConsumer doubleConsumer, double maxAcceleration){
        this(doubleConsumer);
        this.maxAcceleration = maxAcceleration;
    }

    private final ElapsedTime loopTime = new ElapsedTime();
    private double currentSpeed = 0;

    public void setSpeed(double requestedSpeed){
        if(abs(requestedSpeed) > abs(currentSpeed) && requestedSpeed != 0)
            currentSpeed += min(abs(requestedSpeed-currentSpeed), abs(maxAcceleration*loopTime.seconds()))*signum(requestedSpeed-currentSpeed);
        else
            currentSpeed = requestedSpeed;
        doubleConsumer.accept(currentSpeed);
        loopTime.reset();
    }

}
