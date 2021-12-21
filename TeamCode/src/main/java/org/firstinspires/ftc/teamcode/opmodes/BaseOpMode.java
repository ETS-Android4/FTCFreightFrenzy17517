package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.VariablesDashboard;
import org.firstinspires.ftc.teamcode.robot.WoENRobot;

public abstract class BaseOpMode extends LinearOpMode {
    protected final WoENRobot robot = new WoENRobot(this);


    public void startLoop() {
    }

    @Override
    public void waitForStart() {
        while (!isStarted() && !isStopRequested()) {
            startLoop();
        }
        super.waitForStart();
    }

    public abstract void main();

    @Override
    public void runOpMode() {
        switch (VariablesDashboard.TelemetryConfig.telemetryType) {
            case DRIVER_STATION:
                telemetry = super.telemetry;
                break;
            case DASHBOARD:
                telemetry = robot.dashboard.getTelemetry();
                break;
            case DUAL:
                telemetry = new MultipleTelemetry(telemetry, robot.dashboard.getTelemetry());
                break;
        }
        robot.init();
        waitForStart();
        main();
        telemetry.addData("Status","Finished successfully");
        telemetry.update();
    }
}
