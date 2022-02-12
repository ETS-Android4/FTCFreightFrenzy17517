package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.teamcode.opencv.ArucoDetect.centreOfDuck;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.robot.Bucket;
import org.firstinspires.ftc.teamcode.robot.Lift;

@Autonomous
public class AutonomTestOpenCV extends BaseDetectionAutonomous{

    Runnable[] test = {
            ()->{
                robot.telemetryNode.getTelemetry().addData("forceGetPosition",robot.arucoDetect.forceGetPosition()  );
                robot.telemetryNode.getTelemetry().addData("timePosition",robot.arucoDetect.timePosition);
                robot.telemetryNode.getTelemetry().addData("offset", centreOfDuck);
                },
    };

    @Override
    public void main() {
        while (opModeIsActive())
            execute(test);
    }

}
