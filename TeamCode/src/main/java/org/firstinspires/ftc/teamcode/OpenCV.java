package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;


// OpenCV.init(HardwareMap)
// OpenCV.getPosition()
public class OpenCV
{
    enum FreightPosition{
        LEFT, RIGHT,CENTER, UNKNOWN
    }
    private OpenCvWebcam camera;
    private final int width = 320;
    private final int height = 240;
    private int x = 0;
    static final Scalar COLOR_MIN = new Scalar(12, 80, 40);
    static final Scalar COLOR_MAX = new Scalar(18, 255, 255);
    private final org.opencv.core.Size size = new org.opencv.core.Size(5, 5);
    public void init(HardwareMap hwMap)
    {
        int cameraMonitorViewId = hwMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hwMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hwMap.get(WebcamName.class,"Webcam 1"), cameraMonitorViewId);
        Pipeline pipeline = new Pipeline();
        camera.setPipeline(pipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(width,height, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });
    }
    public FreightPosition stopCamera(){
        FreightPosition value = getPosition();
        camera.closeCameraDeviceAsync(() -> {});
        return value;
    }
    public class Pipeline extends OpenCvPipeline
    {
        @Override
        public Mat processFrame(Mat input)
        {
            Rect rect;
            Mat element = Imgproc.getStructuringElement(Imgproc.MORPH_RECT,new Size(35, 10));
            Imgproc.cvtColor(input, input, Imgproc.COLOR_RGB2HSV);
            Imgproc.GaussianBlur(input, input, size, 5);
            Core.inRange(input, COLOR_MIN, COLOR_MAX, input);
            Imgproc.erode(input, input, element);
            Imgproc.dilate(input, input, element);
            rect = Imgproc.boundingRect(input);
            x = rect.x;
            return input;
        }
        public boolean inRange(int down, int up)
        {
            return x > up && x < down;
        }
        public FreightPosition returnPosition()
        {
            if (inRange(0, width / 3)) return FreightPosition.LEFT;
            if (inRange(width / 3, width / 3 * 2)) return FreightPosition.CENTER;
            if (inRange(width / 3 * 2, width)) return FreightPosition.RIGHT;
            return FreightPosition.UNKNOWN;
        }
    }
    public Pipeline pipeline;
    public FreightPosition getPosition()
    {
        return pipeline.returnPosition();
    }
}