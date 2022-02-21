package org.firstinspires.ftc.teamcode.robot;

import static org.firstinspires.ftc.teamcode.robot.Brush.BrushConfig.motorCurrentThreshold;
import static org.firstinspires.ftc.teamcode.robot.Brush.BrushConfig.motorPower;
import static org.firstinspires.ftc.teamcode.robot.Brush.BrushConfig.timeForActivateProtection;
import static org.firstinspires.ftc.teamcode.robot.Brush.BrushConfig.timeForReverse;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.misc.CommandSender;
import org.firstinspires.ftc.teamcode.misc.TimedSensorQuery;


public class Brush implements RobotModule {

    private final WoENRobot robot;
    public ElapsedTime timerProtection = new ElapsedTime();
    private boolean enableIntake = false;
    private DcMotorEx brushMotor = null;

    private final TimedSensorQuery<Double> motorCurrentQuery =
            new TimedSensorQuery<>(() -> brushMotor.getCurrent(CurrentUnit.AMPS), 15);
    private final CommandSender motorPowerSender =
            new CommandSender((double value) -> brushMotor.setPower(value));

    private double brushMotorCurrent = 0;

    public Brush(WoENRobot robot) {
        this.robot = robot;
    }

    private void updateBrushMotorCurrent() {
        brushMotorCurrent = motorCurrentQuery.getValue();
    }

    public double getBrushMotorCurrent() {
        if (enableIntake) return brushMotorCurrent;
        else return 0;
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
        if (brushMotorCurrent > motorCurrentThreshold || (timer > timeForActivateProtection &&
                timer < timeForActivateProtection + timeForReverse)) {
            return (timer >= timeForActivateProtection);
        } else {
            timerProtection.reset();
            return false;
        }
    }

    public boolean getEnableIntake() {
        return enableIntake;
    }

    public void setEnableIntake(boolean intake) {
        this.enableIntake = intake;
    }

    public void update() {
        double brushPower;
        if (enableIntake && robot.lift.getElevatorPosition() == Lift.ElevatorPosition.DOWN &&
                robot.bucket.getBucketPosition() == Bucket.BucketPosition.COLLECT) {
            updateBrushMotorCurrent();
            if (robot.bucket.isFreightDetected() || protectionBrushMotor())
                brushPower = -motorPower;
            else brushPower = motorPower;
        } else {
            brushPower = 0;
        }
        motorPowerSender.send(brushPower);
    }

    public boolean actionIsCompleted() {
        return true;
    }

    @Config
    public static class BrushConfig {
        public static double motorPower = 1;
        public static double motorCurrentThreshold = 3.0;
        public static double timeForActivateProtection = 0.5;
        public static double timeForReverse = /* timeForActivateProtection + */ 0.2;
    }
}
