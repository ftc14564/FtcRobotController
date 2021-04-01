package org.firstinspires.ftc.teamcode.opencv;

import android.util.Log;

import org.opencv.core.Mat;

public class UltimateGoalCVHelper {

    private static final String TAG = "UltimateGoalCVHelper";

    RingDetector ringDetector = new RingDetector();

    public char detectRings(OpenCVHelper openCVHelper)
    {
        char ch_result = '?';
        // get frame from capture device
        Mat mat = openCVHelper.getCameraFrameAsMat();
        try {
            ch_result = ringDetector.process(mat);
        } catch (InterruptedException ex)
        {
            Log.d(TAG, "ringDetector Processing image failed");
        }
        mat.release();
        return ch_result;
    }
}
