package org.firstinspires.ftc.teamcode.robot;

import static org.firstinspires.ftc.teamcode.robot.LedStrip.LedStripConfig.LEDPower;
import static org.firstinspires.ftc.teamcode.robot.LedStrip.LedStripConfig.wavePeriod;
import static java.lang.Math.sin;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.misc.TimedSensorQuery;

public class LedStrip implements RobotModule {

    private LedStripMode ledStripMode = LedStripMode.DRIVER_INDICATOR;
    private DcMotorEx ledMotor = null;
    private final ElapsedTime lightTimer = new ElapsedTime();
    private boolean dualLEDSwitchIterator = false;
    private WoENRobot robot;

    public LedStrip(WoENRobot robot) {
        this.robot = robot;
    }

    public void resetTimer() {
        lightTimer.reset();
    }

    public void setMode(LedStripMode ledStripMode) {
        if (!this.ledStripMode.equals(ledStripMode)) lightTimer.reset();
        this.ledStripMode = ledStripMode;
    }

    @Override
    public void initialize() {
        ledMotor = robot.getLinearOpMode().hardwareMap.get(DcMotorEx.class, "Led");
        ledMotor.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    private double sineWave() {
        return sin(lightTimer.seconds() * wavePeriod);
    }

    private double positiveSineWave() {
        return sineWave() * 0.5 + 0.5;
    }

    private double sawtoothWave() {
        return lightTimer.seconds() % wavePeriod < 1.0 ? 1 : -1;
    }

    private double positiveSawtoothWave() {
        return sawtoothWave() * 0.5 + 0.5;
    }

    TimedSensorQuery<Double> ledCurrentQuery = new TimedSensorQuery<>(()->ledMotor.getCurrent(CurrentUnit.AMPS), 0.5);

    private double ledCurrentValue = 0.0;

    public double getLEDCurrent(){
        return ledCurrentValue;
    }

    @Override
    public void update() {
        double power = 0;
        switch (ledStripMode) {
            case OFF:
                power = .0;
                break;
            case BREATHE_COLOR1:
                power = positiveSineWave();
                break;
            case BREATHE_COLOR2:
                power = -positiveSineWave();
                break;
            case STATIC_COLOR1:
                power = 1.0;
                break;
            case STATIC_COLOR2:
                power = -1.0;
                break;
            case BLINK_COLOR1:
                power = positiveSawtoothWave();
                break;
            case BLINK_COLOR2:
                power = -positiveSawtoothWave();
                break;
            case BLINK_DUALCOLOR:
                power = sawtoothWave();
                break;
            case BREATHE_DUALCOLOR:
                power = sineWave();
                break;
            case BREATHE_MAGIC:
                power = dualLEDSwitchIterator ? sin(lightTimer.seconds() * wavePeriod) * 0.5 + 0.5 :
                        -(sin(lightTimer.seconds() * wavePeriod + 0.75 * Math.PI) * 0.5 + 0.5);
                break;
            case STATIC_DUALCOLOR:
                power = dualLEDSwitchIterator ? 1 : -1;
                break;
            case DRIVER_INDICATOR:
                power = robot.bucket.isFreightDetected() ?
                        ((robot.lift.getElevatorPosition() == Lift.ElevatorPosition.DOWN ? 1 :
                                positiveSineWave())) :
                        (robot.brush.getEnableIntake() ? -positiveSawtoothWave() : 0);
                break;
        }

        dualLEDSwitchIterator = !dualLEDSwitchIterator;
        ledCurrentValue = ledCurrentQuery.getValue();
        ledMotor.setPower(power * LEDPower);
    }

    public enum LedStripMode {
        OFF,
        BREATHE_COLOR1,
        BREATHE_COLOR2,
        STATIC_COLOR1,
        STATIC_COLOR2,
        BLINK_COLOR1,
        BLINK_COLOR2,
        BLINK_DUALCOLOR,
        BREATHE_DUALCOLOR,
        DRIVER_INDICATOR,
        STATIC_DUALCOLOR,
        BREATHE_MAGIC
    }

    public static class LedStripConfig {
        public static double LEDPower = 1;
        public static double wavePeriod = 2.5;
    }
}