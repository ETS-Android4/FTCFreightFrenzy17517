package org.firstinspires.ftc.teamcode.robot;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.hardware.lynx.EmbeddedControlHubModule;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.opencv.ArucoDetect;

import java.util.Arrays;

public class WoENRobot {
    private final LinearOpMode linearOpMode;
    public Bucket bucket = new Bucket(this);
    public LedStrip ledStrip = new LedStrip(this);
    public Duck duck = new Duck(this);
    public Lift lift = new Lift(this);
    public Movement movement = new Movement(this);
    public Brush brush = new Brush(this);
    public Timer timer = new Timer();
    private final RobotModule[] allModules = new RobotModule[]{
            bucket,
            ledStrip,
            duck,
            lift,
            movement,
            brush,
            timer
    };
    public ArucoDetect arucoDetect = new ArucoDetect(this);
    public FtcDashboard dashboard = FtcDashboard.getInstance();

    public WoENRobot(LinearOpMode linearOpMode) {
        this.linearOpMode = linearOpMode;
    }

    public LinearOpMode getLinearOpMode() {
        return linearOpMode;
    }

    public void init() {
        for (RobotModule robotModule : allModules)
            robotModule.initialize();
    }

    public void update() {
        for (RobotModule robotModule : allModules)
            robotModule.update();
    }

    public boolean allActionsAreCompleted() {
        return Arrays.stream(allModules).allMatch(RobotModule::actionIsCompleted);
    }

}
