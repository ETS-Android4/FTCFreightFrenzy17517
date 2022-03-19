package org.firstinspires.ftc.teamcode;

import static java.lang.Math.PI;

public class WoENMath {

    public static double angleWrap(double angle) {
        while (angle > PI) angle -= PI * 2;
        while (angle < -PI) angle += PI * 2;
        return angle;
    }

    public static double angleWrapHalf(double angle) {
        while (angle > PI / 2) angle -= PI;
        while (angle < -PI / 2) angle += PI;
        return angle;
    }

}
