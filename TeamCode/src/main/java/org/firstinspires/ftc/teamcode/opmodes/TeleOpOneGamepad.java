package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.teamcode.VariablesDashboard.TeleOpConfig.robotSpeed;
import static org.firstinspires.ftc.teamcode.robot.Elevator.ElevatorPosition.UP;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.misc.ButtonOperations.SmartButtonSwitch;
import org.firstinspires.ftc.teamcode.misc.ButtonActivatedModes.ButtonActivated;
import org.firstinspires.ftc.teamcode.misc.ButtonOperations.ButtonSwitch;
import org.firstinspires.ftc.teamcode.robot.Brush;
import org.firstinspires.ftc.teamcode.robot.Duck;
import org.firstinspires.ftc.teamcode.robot.Elevator;
import org.firstinspires.ftc.teamcode.robot.RobotModules;
import org.firstinspires.ftc.teamcode.robot.Sensor_system;

@TeleOp
public class TeleOpOneGamepad extends LinearOpMode {
    private final RobotModules robotModules = new RobotModules(this);
    private Duck duck = new Duck(this);
    private SmartButtonSwitch duck_function = new SmartButtonSwitch(() -> gamepad1.circle,(Boolean duckb) -> robotModules.duck.DuckSpin(duckb));
    private SmartButtonSwitch servo_elevator_function = new SmartButtonSwitch(() -> gamepad1.square,(Boolean elev) -> robotModules.elevator.MoveServoForElevator(elev));
    private SmartButtonSwitch intake_function = new SmartButtonSwitch(() -> gamepad1.triangle,(Boolean intake) -> robotModules.brush.brushMotorMove(intake));
    private ButtonSwitch buttonSwitch = new ButtonSwitch();
    private ButtonSwitch buttonSwitch1 = new ButtonSwitch();
   // private Sensor_system sensor_system = new Sensor_system(this);
    private Brush getBrush = new Brush(this);


    public ButtonActivated BA;
    private Brush brush;

    public boolean cube_bool_1 = false;
    public boolean cube_bool_2 = false;

    public boolean t = true;
    public boolean u = true;
    public double x = 0.0;
    public double y = 0.0;
    public double OX = 0.0;
    public double OY = 0.0;
    public double pl = 0.0;

    @Override
    public void runOpMode() {

        robotModules.init();
        RobotModules.duck.Teleop();

        waitForStart();
        while (opModeIsActive()) {
            // Local conditions
            if(gamepad1.left_bumper){ obnul(u); }
            RobotModules.movement.setMotorPowers(-gamepad1.left_stick_y * get_speed(), gamepad1.right_stick_x * pl);
            // Switch functions
            duck_function.activate();
            servo_elevator_function.activate();
            /*intake_function.activate();*/
            // Others
            RobotModules.brush.brushMotorMove(t && buttonSwitch.getState(gamepad1.triangle));
            lift_function();
           // cube_fix(true);
            robotModules.updateForTeleop();
            /*Drawing();*/
        }
    }

    /*private void cube_fix(boolean activation_bool){
        if(activation_bool) {
            if (brush.freightIsDetected) {
                getBrush.ledMotor.setPower(1.0);
                cube_bool_1 = true;
            } else {
                getBrush.ledMotor.setPower(0.0);
                cube_bool_2 = true;
            }
        }
    }*/

    private double get_speed(){ if(buttonSwitch1.getState(gamepad1.right_bumper)){ pl = 1; } else{ pl = 0.5; } return pl; }

    private void obnul(boolean i){
        if(i){ while(gamepad1.left_bumper && !RobotModules.elevator.limitSwitch.getState()){ robotModules.elevator.motorLift.setPower(-1); u = false; } robotModules.elevator.resetEncoderElevator();}
    }

    private void lift_function(){
        if(gamepad1.dpad_up || cube_bool_1){ cube_bool_1 = false; RobotModules.elevator.ElevatorPosition(UP); t = false;}
        if(gamepad1.dpad_left){ RobotModules.elevator.ElevatorPosition(Elevator.ElevatorPosition.MIDDLE); t = false;}
        if(gamepad1.dpad_down || cube_bool_2){ cube_bool_2 = false; RobotModules.elevator.ElevatorPosition(Elevator.ElevatorPosition.DOWN); t = true;}
    }

    /*public void Drawing() {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("x", x);
        packet.put("y", y);
        packet.fieldOverlay()
                .setFill("red")
                .fillRect(OX / 1000, OY / 1000, 40, 40);
        dashboard.sendTelemetryPacket(packet);
    }*/
}
