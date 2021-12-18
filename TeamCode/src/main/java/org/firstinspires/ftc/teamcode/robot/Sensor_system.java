package org.firstinspires.ftc.teamcode.robot;


import static org.firstinspires.ftc.teamcode.VariablesDashboard.Duck.directionDuck;
import static org.firstinspires.ftc.teamcode.VariablesDashboard.ManipulatorConfig.positonServoForElevator;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorColor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.misc.ButtonActivatedModes.ButtonActivated;

public class Sensor_system {
    private final LinearOpMode linearOpMode;
    private SensorColor SC = new SensorColor();
    private DistanceSensor distanceSensor = null;

    public Sensor_system(LinearOpMode linearOpMode){ this.linearOpMode = linearOpMode; }

    public void init(){
        distanceSensor = linearOpMode.hardwareMap.get(DistanceSensor.class, "distance");
        distanceSensor.resetDeviceConfigurationForOpMode();
    }

    public boolean block_status(){
        if(distanceSensor.getDistance(DistanceUnit.CM) <= 8){ return true; }
        else{ return false; }
    }
}
