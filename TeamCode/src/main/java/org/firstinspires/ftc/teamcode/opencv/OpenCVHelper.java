package org.firstinspires.ftc.teamcode.opencv;

import android.graphics.Bitmap;
import android.util.Log;

import org.opencv.android.LoaderCallbackInterface;
import org.opencv.android.OpenCVLoader;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.vuforia.Image;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.opencv.android.BaseLoaderCallback;
import org.opencv.android.Utils;
import org.opencv.core.CvType;
import org.opencv.core.Mat;

import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

public class OpenCVHelper {
    private static final String TAG = "OpenCVHelper";

    private static final String VUFORIA_KEY =
            "AYpOJ0H/////AAABGeEbm+5m+k5BrTnPlF3X9R177NGoUFUGl1kpgLa7MBwlsRdnD3IdxY7LmZ41NTQMASZ1MbCWaEpM4Sag7tDfQsJjqVvCwZr3qJm5y33J8rnMWz1ViOwwzZgnsSZqeGRY9+uPGa6cTMO/cxs+YF+4OqsD+iu4exeMCsxyAPYhXQrEIaW6h7zYVrdi9b5WsgNGUfP60Qz8U3szKTfVmaHmMFvc+iuJ1qmAM5AjlsBlc8MMHzLAL/3sf3UiCDe4tgo4mmYEsdl499QhqhhImEiKS8rTkap/53B8Hm89z3m5HuBoH4EKVUc65k2aCBg5c5jXVoZan8DkQFqSPnArwQnCHpaL/d1y79BRE44nJXj54E6V";


    VuforiaLocalizer vuforia;
    AppUtil appUtil = AppUtil.getInstance();

    private BaseLoaderCallback loaderCallback = new BaseLoaderCallback(appUtil.getActivity())
    {
        @Override
        public void onManagerConnected(int status) {
            super.onManagerConnected(status);
        }
    };


    public VuforiaLocalizer initializeOpenCVAndVuforiaCamera(HardwareMap hardwareMap, String cameraDeviceName, VuforiaLocalizer.CameraDirection cameraChoice, boolean phoneIsPortrait)
    {
        initailizeOpenCV();
        return initializeVuforiaCamera( hardwareMap,  cameraDeviceName, cameraChoice,  phoneIsPortrait);
    }


    public void initailizeOpenCV()
    {
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
    }

    public VuforiaLocalizer initializeVuforiaCamera(HardwareMap hardwareMap, String cameraDeviceName, VuforiaLocalizer.CameraDirection cameraChoice, boolean phoneIsPortrait)
    {
        // cameraDeviceName = "Internal";  // valid values are "Internal", "Webcam 1", "Webcam 2"
        WebcamName webcamName = null;

        if (cameraDeviceName.startsWith("Webcam"))
        {
            webcamName = hardwareMap.get(WebcamName.class, cameraDeviceName);
        }

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection   = cameraChoice;

        if (webcamName != null)
        {
            parameters.cameraName = webcamName;
        }

        // Make sure extended tracking is disabled for this example.
        parameters.useExtendedTracking = false;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        Vuforia.setFrameFormat(PIXEL_FORMAT.RGB565, true);
        vuforia.setFrameQueueCapacity(1);

        Log.d(TAG, "created vuforia obj");
        return vuforia;

    }

    public Mat getCameraFrameAsMat()
    {
        VuforiaLocalizer.CloseableFrame frame = null;
        Mat mat = null;
        try {
            frame = vuforia.getFrameQueue().take();
        } catch (InterruptedException ex)
        {
            Log.v(TAG, "could note TAKE image");
        }
        char ch_result = '?';
        long numImages = frame.getNumImages();
        for (int i = 0; i < numImages; i++) {
            if (frame.getImage(i).getFormat() == PIXEL_FORMAT.RGB565) {
                Image rgb = frame.getImage(i);
                mat = new Mat(rgb.getHeight(), rgb.getWidth(), CvType.CV_8UC3);

                if (rgb != null) {
                    Bitmap bm = Bitmap.createBitmap(rgb.getWidth(), rgb.getHeight(), Bitmap.Config.RGB_565);
                    bm.copyPixelsFromBuffer(rgb.getPixels());
                    Utils.bitmapToMat(bm, mat);

                    //mat.release();
                    frame.close();
                    Log.d(TAG, "Ring = " + String.valueOf(ch_result));
                    break;  // one frame is all we are setup to get
                }
            }
        }
        return mat;
    }
}
