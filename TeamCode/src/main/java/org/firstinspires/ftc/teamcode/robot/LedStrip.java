package org.firstinspires.ftc.teamcode.robot;

import static java.lang.Math.sin;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

public class LedStrip implements RobotModule {

    private LedStripMode ledStripMode = LedStripMode.INDICATOR;
    private DcMotorEx ledMotor = null;
    private ElapsedTime time = new ElapsedTime();
    private WoENRobot robot = null;

    public LedStrip(WoENRobot robot) {
        this.robot = robot;
    }

    public void resetTimer() {
        time.reset();
    }

    public void setMode(LedStripMode ledStripMode) {
        if (this.ledStripMode != ledStripMode)
            time.reset();
            this.ledStripMode = ledStripMode;
    }

    @Override
    public void initialize() {
        ledMotor = robot.getLinearOpMode().hardwareMap.get(DcMotorEx.class, "Led");
        ledMotor.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    @Override
    public void update() {
        double power = 0;
        switch (ledStripMode) {
            case OFF:
                power = .0;
                break;
            case BREATHE:
                power = sin(time.seconds() * 2.0) * 0.5 + 0.5;
                break;
            case STATIC:
                power = 1.0;
                break;
            case BLINK:
                power = time.seconds() % 2.0 < 1.0 ? 1 : 0;
                break;
            case BLINK_DUALCOLOR:
                power = time.seconds() % 2.0 < 1.0 ? 1 : -1;
                break;
            case BREATHE_DUALCOLOR:
                power = sin(time.seconds() * 2.0);
                break;
            case INDICATOR:
                break;
        }
        ledMotor.setPower(power);
    }

    public enum LedStripMode {
        OFF, BREATHE, STATIC, BLINK, BLINK_DUALCOLOR, BREATHE_DUALCOLOR, INDICATOR
    }
}