package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
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
        robot.init();
        waitForStart();
        main();
        telemetry.addData("Status","Finished successfully");
        telemetry.update();
    }
}
