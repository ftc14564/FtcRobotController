package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;
import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.vuforia.Image;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.opencv.RingDetector;
import org.opencv.android.BaseLoaderCallback;
import org.opencv.android.LoaderCallbackInterface;
import org.opencv.android.OpenCVLoader;
import org.opencv.android.Utils;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.videoio.VideoCapture;
import org.opencv.videoio.Videoio;

import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

@Autonomous(name = "RingAutonomous")
public class RingAutonomous extends  Teleop2021 {

    private static final String TAG = "RingAutonomous";
    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;
    private static final boolean PHONE_IS_PORTRAIT = false  ;

    private static final String VUFORIA_KEY =
            "AYpOJ0H/////AAABGeEbm+5m+k5BrTnPlF3X9R177NGoUFUGl1kpgLa7MBwlsRdnD3IdxY7LmZ41NTQMASZ1MbCWaEpM4Sag7tDfQsJjqVvCwZr3qJm5y33J8rnMWz1ViOwwzZgnsSZqeGRY9+uPGa6cTMO/cxs+YF+4OqsD+iu4exeMCsxyAPYhXQrEIaW6h7zYVrdi9b5WsgNGUfP60Qz8U3szKTfVmaHmMFvc+iuJ1qmAM5AjlsBlc8MMHzLAL/3sf3UiCDe4tgo4mmYEsdl499QhqhhImEiKS8rTkap/53B8Hm89z3m5HuBoH4EKVUc65k2aCBg5c5jXVoZan8DkQFqSPnArwQnCHpaL/d1y79BRE44nJXj54E6V";


    AppUtil appUtil = AppUtil.getInstance();

    private BaseLoaderCallback loaderCallback = new BaseLoaderCallback(appUtil.getActivity())
    {
        @Override
        public void onManagerConnected(int status) {
            super.onManagerConnected(status);
        }
    };

    @Override
    public void runOpMode()  {

        // get videocapture
        Log.v(TAG, "Initializing OpenCV");
        // Initialize OpenCV
        if (!OpenCVLoader.initDebug()) {
            Log.d(TAG, "Internal OpenCV library not found. Using OpenCV Manager for initialization");
            OpenCVLoader.initAsync(OpenCVLoader.OPENCV_VERSION_3_1_0, appUtil.getActivity(), loaderCallback);
        }
        else {
            Log.d(TAG, "OpenCV library found inside package. Using it!");
            loaderCallback.onManagerConnected(LoaderCallbackInterface.SUCCESS);
        }

        Log.v(TAG, "Initialized OpenCV");


        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection   = CAMERA_CHOICE;

        // Make sure extended tracking is disabled for this example.
        parameters.useExtendedTracking = false;

        //  Instantiate the Vuforia engine
        VuforiaLocalizer vuforia = ClassFactory.getInstance().createVuforia(parameters);

        Vuforia.setFrameFormat(PIXEL_FORMAT.RGB565, true);
        vuforia.setFrameQueueCapacity(1);

        Log.d(TAG, "created vuforia obj");

        RingDetector ringDetector = new RingDetector();
        telemetry.update();
        VuforiaLocalizer.CloseableFrame frame = null;
        telemetry.addData("Waiting for start", "Click it");
        waitForStart();
        Log.d(TAG, "start happened");

        while (opModeIsActive()) {
            // get frame from capture device
            //Log.d(TAG, "in loop");
            //telemetry.addData("Loop started", "clicked");

            try {
                frame = vuforia.getFrameQueue().take();
            } catch (InterruptedException ex)
            {
                Log.v(TAG, "could note TAKE image");
            }
            //Log.v(TAG, "***** Got a frame or more");


            char ch_result = '?';

            long numImages = frame.getNumImages();
            for (int i = 0; i < numImages; i++) {
                if (frame.getImage(i).getFormat() == PIXEL_FORMAT.RGB565) {
                    Image rgb = frame.getImage(i);
                    Mat mat = new Mat(rgb.getHeight(), rgb.getWidth(), CvType.CV_8UC3);

                    if (rgb != null) {
                        Bitmap bm = Bitmap.createBitmap(rgb.getWidth(), rgb.getHeight(),
                                Bitmap.Config.RGB_565);
                        bm.copyPixelsFromBuffer(rgb.getPixels());
                        Utils.bitmapToMat(bm, mat);
                        try {
                            ch_result = ringDetector.process(mat);
                        } catch (InterruptedException ex)
                        {
                            Log.d(TAG, "ringDetector Processing image failed");
                        }
                        mat.release();
                        telemetry.addData("Ring Detected", "Path = %s", String.valueOf(ch_result));
                        Log.d(TAG, "Ring = " + String.valueOf(ch_result));
                        telemetry.update();
                        //break;  // one frame is plenty
                    }
                }
            }

            telemetry.update();
            idle();
        }
        telemetry.addData("Ring Detection Termination", "");

        // close camera

        telemetry.update();
    }



}
