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


// OpenCV.Init(HardwareMap)
// int i = OpenCV.GetPosition()
// i = 1 : Left
// i = 2 : Center
// i = 3 : Right
// i = 0 : Not Found
public class OpenCV
{
    private static OpenCvWebcam phoneCam;
    private static final int widht = 320;
    private static final int height = 240;
    private static double x = 0.0;
    static final Scalar COLOR_MIN = new Scalar(12, 80, 40);
    static final Scalar COLOR_MAX = new Scalar(18, 255, 255);
    public static void Init(HardwareMap hwMap)
    {
        int cameraMonitorViewId = hwMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hwMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createWebcam(hwMap.get(WebcamName.class,"Webcam 1"), cameraMonitorViewId);
        Pipeline pipeline = new Pipeline();
        phoneCam.setPipeline(pipeline);
        phoneCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                phoneCam.startStreaming(widht,height, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });
    }
    public static class Pipeline extends OpenCvPipeline
    {
        @Override
        public Mat processFrame(Mat input)
        {
            Rect rect;
            Mat element = Imgproc.getStructuringElement(Imgproc.MORPH_RECT,new Size(35, 10));
            Imgproc.cvtColor(input, input, Imgproc.COLOR_RGB2HSV);
            Imgproc.GaussianBlur(input, input, new  org.opencv.core.Size(5, 5), 5);
            Core.inRange(input, COLOR_MIN, COLOR_MAX, input);
            Imgproc.erode(input, input, element);
            Imgproc.dilate(input, input, element);
            rect = Imgproc.boundingRect(input);
            x = rect.x;
            return null;
        }
        public boolean InRange(int down, int up)
        {
            return x > up && x < down;
        }
        public int ReturnPosition()
        {
            if (InRange(0, widht / 3)) return 1;
            if (InRange(widht / 3, widht / 3 * 2)) return 2;
            if (InRange(widht / 3 * 2, widht)) return 3;
            return 0;
        }
    }
    public Pipeline pipeline;
    public int GetPosition()
    {
        return pipeline.ReturnPosition();
    }
}