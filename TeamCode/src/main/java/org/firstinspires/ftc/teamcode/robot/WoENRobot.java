package org.firstinspires.ftc.teamcode.robot;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.opencv.ArucoDetect;

import java.util.Arrays;
import java.util.List;

public class WoENRobot {
    public final Bucket bucket = new Bucket(this);
    public final LedStrip ledStrip = new LedStrip(this);
    public final Duck duck = new Duck(this);
    public final Lift lift = new Lift(this);
    public final Movement movement = new Movement(this);
    public final DifferentialOdometry odometry = new DifferentialOdometry(this);
    public final Brush brush = new Brush(this);
    public final Timer timer = new Timer();
    public final TelemetryNode telemetryNode = new TelemetryNode(this);
    public final Battery battery = new Battery(this);
    public final Gyro gyro = new Gyro(this);
    public final GyroAuto gyroAuto = new GyroAuto(this);
    private final LinearOpMode linearOpMode;
    private final RobotModule[] allModules = new RobotModule[]{bucket,
            gyro,
            ledStrip,
            duck,
            lift,
            movement,
            odometry,
            brush,
            timer,
            battery,
            gyroAuto,
            telemetryNode};
    public ArucoDetect arucoDetect = new ArucoDetect(this);
    public FtcDashboard dashboard = FtcDashboard.getInstance();
    private List<LynxModule> revHubs = null;

    public WoENRobot(LinearOpMode linearOpMode) {
        this.linearOpMode = linearOpMode;
    }

    public LinearOpMode getLinearOpMode() {
        return linearOpMode;
    }

    public void init() {
        revHubs = linearOpMode.hardwareMap.getAll(LynxModule.class);
        for (RobotModule robotModule : allModules)
            robotModule.initialize();
        for (LynxModule module : revHubs)
            module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
    }

    public void updateRevBulkCache() {
        for (LynxModule module : revHubs)
            module.clearBulkCache();
    }

    public void update() {
        updateRevBulkCache();
        for (RobotModule robotModule : allModules)
            robotModule.update();
    }

    public boolean allActionsAreCompleted() {
        return Arrays.stream(allModules).allMatch(RobotModule::actionIsCompleted);
    }

}
