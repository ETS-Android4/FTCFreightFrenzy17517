package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.VariablesDash.*;
import static org.firstinspires.ftc.teamcode.Movement.*;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

@Autonomous
public class Autonom extends LinearOpMode {

    FtcDashboard dashboard;

    //creating variables
    private Movement movement = new Movement(this);

    @Override
    public void runOpMode() {

        dashboard = FtcDashboard.getInstance();

        movement.init();

        waitForStart();
        while (opModeIsActive()){
            movement.Move(1000);
            movement.Move(0);
        }
    }
}