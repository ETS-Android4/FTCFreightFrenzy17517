package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.opmode.TelemetryImpl;

public class Tele {
    private LinearOpMode linearOpMode = null;
    public Tele(LinearOpMode linearOpMode){
        this.linearOpMode = linearOpMode;
    }
    public Telemetry telemetry = new TelemetryImpl(linearOpMode);
    private Encoders encoders = new Encoders(linearOpMode);

    public void activate() {
        telemetry.addData("EncRight",encoders.Get_2());
        telemetry.addData("EncLeft",encoders.Get_1());
        telemetry.addData("Angle",encoders.get_angle());
        telemetry.update();
    }
}
