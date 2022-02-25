package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.hardware.bosch.BNO055IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.misc.TimedSensorQuery;

public class Gyro implements RobotModule {

    private final WoENRobot robot;

    private BNO055IMU gyro;

    private final TimedSensorQuery<Orientation> timedGyroQuery = new TimedSensorQuery<>(() -> gyro
            .getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES), 100);

    private Orientation orientation = new Orientation();

    public Gyro(WoENRobot robot) {
        this.robot = robot;
    }

    @Override
    public void initialize() {
        gyro = robot.getLinearOpMode().hardwareMap.get(BNO055IMU.class, "imu");
        gyro.initialize(new BNO055IMU.Parameters());
        orientation = timedGyroQuery.getValue();
    }

    @Override
    public void update() {
        orientation = timedGyroQuery.getValue();
    }

    public Orientation getOrientation() {
        return orientation;
    }
}
