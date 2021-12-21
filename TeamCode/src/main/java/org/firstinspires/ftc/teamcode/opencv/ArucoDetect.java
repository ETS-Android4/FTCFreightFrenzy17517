package org.firstinspires.ftc.teamcode.opencv;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.misc.FreightPosition;
import org.firstinspires.ftc.teamcode.robot.WoENRobot;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

public class ArucoDetect {

    static final double FEET_PER_METER = 3.28084;
    final float DECIMATION_HIGH = 3;
    final float DECIMATION_LOW = 2;
    final float THRESHOLD_HIGH_DECIMATION_RANGE_METERS = 1.0f;
    final int THRESHOLD_NUM_FRAMES_NO_DETECTION_BEFORE_LOW_DECIMATION = 4;
    public double timePosition = 0;
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;
    double tagsize = 0.166;
    int numFramesWithoutDetection = 0;
    private WoENRobot robot;
    public ArucoDetect(WoENRobot robot) {
        this.robot = robot;
    }

    public void initialize() {
        camera = OpenCvCameraFactory.getInstance().createWebcam(robot.getLinearOpMode().hardwareMap.get(WebcamName.class, "Webcam 1"));
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);
        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(800, 448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
            }
        });
    }

    public FreightPosition getPosition() {
        ArrayList<AprilTagDetection> detections = aprilTagDetectionPipeline.getDetectionsUpdate();
        if (detections != null) {
            if (detections.size() == 0) {
                numFramesWithoutDetection++;
                if (numFramesWithoutDetection >= THRESHOLD_NUM_FRAMES_NO_DETECTION_BEFORE_LOW_DECIMATION) {
                    aprilTagDetectionPipeline.setDecimation(DECIMATION_LOW);
                }
                return FreightPosition.UNKNOWN;
            } else {
                numFramesWithoutDetection = 0;
                if (detections.get(0).pose.z < THRESHOLD_HIGH_DECIMATION_RANGE_METERS) {
                    aprilTagDetectionPipeline.setDecimation(DECIMATION_HIGH);
                }
                timePosition = detections.get(0).pose.x * FEET_PER_METER * detections.get(0).pose.z * FEET_PER_METER;
            }
        } else {
            return FreightPosition.UNKNOWN;
        }
        if (timePosition < -10) return FreightPosition.LEFT;
        if (timePosition >= -10 && timePosition < 10) return FreightPosition.CENTER;
        if (timePosition >= 10) return FreightPosition.RIGHT;
        return FreightPosition.UNKNOWN;
    }

    public boolean inRange(double down, double up) {
        return timePosition > up && timePosition < down;
    }

    public FreightPosition stopCamera() {
        FreightPosition value = getPosition();
        camera.closeCameraDeviceAsync(() -> {
        });
        return value;
    }

}

