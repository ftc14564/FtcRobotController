package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opencv.RingDetector;
import org.opencv.android.OpenCVLoader;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.videoio.VideoCapture;

@Autonomous(name = "RingAutonomous")
public class RingAutonomous extends  Teleop2021 {

    RingDetector ringDetector = new RingDetector();
    protected int intCameraIndex = 0;
    private static final String TAG = "RingAutonomous";

    @Override
    public void runOpMode() {
        // get videocapture

        telemetry.addData("Ring Detection Initialization", "Cam ID : %d" , intCameraIndex);
        //OpenCVLoader.initAsync(OpenCVLoader.OPENCV_VERSION_3_0_0, this, mLoaderCallback);
        Log.d(TAG, "Initializing OpenCV");
        OpenCVLoader.initDebug();
        Log.d(TAG, "Initialized OpenCV");

        //System.loadLibrary(Core.NATIVE_LIBRARY_NAME);
        VideoCapture capture = new VideoCapture(intCameraIndex);
        Log.d(TAG, "Videocapture");

        Mat matImage = new Mat();

        if (! capture.isOpened())
        {
            telemetry.addData("Wrong camera?", ":(");
        }

        while (opModeIsActive()) {
            // idle();
            // get frame from capture device
            char ch_result = '?';
            capture.read(matImage);
            try {
                ch_result = ringDetector.process(matImage);
            } catch (InterruptedException ex)
            {
                // handle exception
            }
            // telemetry the result
            telemetry.addData("Ring Detected", "Path = %s", String.valueOf(ch_result));
        }
        telemetry.addData("Ring Detection Termination", "");

        // close camera
        capture.release();
    }



}
