package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.HardwareMap;

public interface RobotModule {
    void initialize();

    void update();

    default boolean actionIsCompleted() {
        return true;
    }
}
