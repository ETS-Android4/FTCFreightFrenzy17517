package org.firstinspires.ftc.teamcode.robot;

import static org.firstinspires.ftc.teamcode.robot.Brush.BrushConfig.timeForActivateProtection;
import static org.firstinspires.ftc.teamcode.robot.Brush.BrushConfig.timeForReverse;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;


public class Brush implements RobotModule {

    private final WoENRobot robot;
    public ElapsedTime timerProtection = new ElapsedTime();
    private boolean enableIntake = false;
    private DcMotorEx brushMotor = null;

    public Brush(WoENRobot robot) {
        this.robot = robot;
    }

    public void initialize() {

        brushMotor = robot.getLinearOpMode().hardwareMap.get(DcMotorEx.class, "BrushMotor");
        brushMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        brushMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        brushMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        brushMotor.setDirection(DcMotorEx.Direction.REVERSE);

    }

    public boolean protectionBrushMotor() {
        double timer = timerProtection.seconds();
        if (brushMotor.getCurrent(CurrentUnit.MILLIAMPS) > 3000 ||
                (timer > timeForActivateProtection && timer < timeForActivateProtection + timeForReverse)) {
            return (timer >= timeForActivateProtection);
        } else {
            timerProtection.reset();
            return false;
        }
    }

    public void enableIntake(boolean intake) {
        this.enableIntake = intake;
    }

    public boolean getEnableIntake() {
        return enableIntake;
    }

    public void update() {
        double brushPower;
        if (enableIntake && robot.lift.getElevatorPosition() == Lift.ElevatorPosition.DOWN &&
                robot.bucket.getBucketPosition() == Bucket.BucketPosition.COLLECT) {
            if (robot.bucket.isFreightDetected() || protectionBrushMotor()) brushPower = -1;
            else brushPower = 1;
        } else {
            brushPower = 0;
        }
        brushMotor.setPower(brushPower);
    }

    public boolean actionIsCompleted() {
        return true;
    }

    @Config
    public static class BrushConfig{
        public static double timeForActivateProtection = 3.5;
        public static double timeForReverse = /* timeForActivateProtection + */ 1;
    }
}
