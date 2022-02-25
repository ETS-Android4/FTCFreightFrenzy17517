package org.firstinspires.ftc.teamcode.robot;

public interface RobotModule {
    void initialize();

    default void update() {
    }

    default boolean actionIsCompleted() {
        return true;
    }
}
