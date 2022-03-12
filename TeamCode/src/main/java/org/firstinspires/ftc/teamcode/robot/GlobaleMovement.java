package org.firstinspires.ftc.teamcode.robot;

import static java.lang.Math.PI;
import static java.lang.Math.abs;
import static java.lang.Math.signum;
import static java.lang.Math.sqrt;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.misc.CommandSender;


public class GlobaleMovement implements RobotModule{


private WoENRobot robot = null;
private DcMotorEx leftMotorFront = null;
private DcMotorEx leftMotorBack = null;
private final CommandSender leftMotorCommandSender = new CommandSender((double value) -> {
     leftMotorBack.setPower(value);
     leftMotorFront.setPower(value);
});
private DcMotorEx rightMotorFront = null;
private DcMotorEx rightMotorBack = null;
private final CommandSender rightMotorCommandSender = new CommandSender((double value) -> {
      rightMotorBack.setPower(value);
      rightMotorFront.setPower(value);
});


public void update(){

}
public boolean actionIsCompleted(){

    return true;
}
public void initialize(){

}
    public double getXCoordinate(){
        return robot.odometry.getCurrentPosition().x;
    }
    public double getYCoordinate(){
        return robot.odometry.getCurrentPosition().y;
    }
    public double getACoordinate(){
        return robot.odometry.getCurrentPosition().heading;
    }
    public void globalMove(double xTarget, double yTarget, double aTarget){
        double pKAngle = 0;
        double pKDistance = 0;
        pKDistance = 0.2;
        pKAngle = 0.6;
        double aPower = aError(aTarget);
        double dPower = sqrt(xError(xTarget)*xError(xTarget) + yError(yTarget)*yError(yTarget));
        setMotorPowersPrivate(dPower * pKDistance, aPower * pKAngle);
    }
    public double xError(double x){
        return x - getXCoordinate();
    }
    public double yError(double y){
        return y - getYCoordinate();
    }
    public double aError(double a){
        return a - getACoordinate();
    }
    private void setMotorPowersPrivate(double power, double angle) {
        rightMotorCommandSender.send(power - angle);
        leftMotorCommandSender.send(power + angle);
    }


}
