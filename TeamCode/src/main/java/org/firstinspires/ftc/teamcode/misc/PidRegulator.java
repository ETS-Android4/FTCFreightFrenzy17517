package org.firstinspires.ftc.teamcode.misc;

import com.qualcomm.robotcore.util.ElapsedTime;

public class PidRegulator {

    private ElapsedTime cycleTimer = new ElapsedTime();

    private double kP = 0;
    private double kI = 0;
    private double kD = 0;

    public double getPgain() {
        return pGain;
    }

    private double pGain = 0;

    public double getIgain() {
        return iGain;
    }

    private double iGain = 0;

    public double getDgain() {
        return dGain;
    }

    private double dGain = 0;

    double previousError = 0;

    public PidRegulator(double kP, double kI, double kD){
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }

    public void setCoefficients(double kP, double kI, double kD){
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }

    public void reset(){
        cycleTimer.reset();
        iGain = 0;
        previousError = 0;
    }

    public double update(double error){
        double timestep = cycleTimer.seconds();
        cycleTimer.reset();
        pGain = error * kP;
        iGain = (error + previousError)*0.5*timestep;
        dGain = (error - previousError)/timestep;
        previousError = error;
        return pGain + iGain + dGain;
    }


}
