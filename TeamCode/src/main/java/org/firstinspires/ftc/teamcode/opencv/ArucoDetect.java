package org.firstinspires.ftc.teamcode.opencv;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.misc.FreightPosition;
import org.firstinspires.ftc.teamcode.misc.TimedSensorQuery;
import org.firstinspires.ftc.teamcode.robot.WoENRobot;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

public class ArucoDetect {
    static final double FEET_PER_METER = 3.28084;
    public static double entreDistance = 13.5;
    public static double centreOfDuck = 0;
    public static double timePosition = 0;
    final float DECIMATION_HIGH = 3;
    final float DECIMATION_LOW = 2;
    final float THRESHOLD_HIGH_DECIMATION_RANGE_METERS = 1.0f;
    final int THRESHOLD_NUM_FRAMES_NO_DETECTION_BEFORE_LOW_DECIMATION = 4;
    private final WoENRobot robot;
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;
    double tagsize = 0.166;
    int numFramesWithoutDetection = 0;
    private final TimedSensorQuery<FreightPosition> freightPositionTimedSensorQuery =
            new TimedSensorQuery<>(this::forceGetPosition, 1.5);

    public ArucoDetect(WoENRobot robot) {
        this.robot = robot;
    }

    public void initialize() {
        camera = OpenCvCameraFactory.getInstance()
                .createWebcam(robot.getLinearOpMode().hardwareMap.get(WebcamName.class, "Webcam 1"));
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);
        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(800, 448, OpenCvCameraRotation.UPRIGHT);
            //    FtcDashboard.getInstance().startCameraStream(camera, 5);
            }

            @Override
            public void onError(int errorCode) {
            }
        });
    }

    public FreightPosition forceGetPosition() {
        ArrayList<AprilTagDetection> detections = aprilTagDetectionPipeline.getDetectionsUpdate();
        if (detections != null) {
            if (detections.size() == 0) {
                numFramesWithoutDetection++;
                if (numFramesWithoutDetection >= THRESHOLD_NUM_FRAMES_NO_DETECTION_BEFORE_LOW_DECIMATION) {
                    aprilTagDetectionPipeline.setDecimation(DECIMATION_LOW);
                }
                return FreightPosition.RIGHT;
            } else {
                numFramesWithoutDetection = 0;
                if (detections.get(0).pose.z < THRESHOLD_HIGH_DECIMATION_RANGE_METERS) {
                    aprilTagDetectionPipeline.setDecimation(DECIMATION_HIGH);
                }
                timePosition = detections.get(0).pose.x * FEET_PER_METER * detections.get(0).pose.z * FEET_PER_METER;
                timePosition = timePosition + centreOfDuck;
            }
        } else {
            return FreightPosition.RIGHT;
        }
        if (timePosition < -entreDistance) return FreightPosition.LEFT;
        if (timePosition >= -entreDistance && timePosition < entreDistance) return FreightPosition.CENTER;
        return FreightPosition.RIGHT;
    }

    public FreightPosition getPosition() {
        return freightPositionTimedSensorQuery.getValue();
    }

    public FreightPosition stopCamera() {
        FreightPosition value = forceGetPosition();
        //FtcDashboard.getInstance().stopCameraStream();
        //camera.stopStreaming();
        camera.closeCameraDeviceAsync(()->{});
        return value;
    }

    @Config
    public static class OpenCVConfig {
        public static boolean doCameraStream = false;
    }
}

