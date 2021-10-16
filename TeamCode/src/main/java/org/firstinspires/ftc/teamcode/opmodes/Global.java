package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Movement;
import org.firstinspires.ftc.teamcode.opmodes.ButtonSwitch;
import org.firstinspires.ftc.teamcode.opmodes.Duck;

public class Global {
    public LinearOpMode linearOpMode = null;
    public Global(LinearOpMode linearOpMode){
        this.linearOpMode = linearOpMode;
    }
    private Duck duck = new Duck(linearOpMode);
    private Movement move = new Movement(linearOpMode);

    public void init(){
        duck.init();
        move.init();

    }
}
