package org.firstinspires.ftc.teamcode.opencv;

import android.util.Log;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;


public class RingDetector  {
    // Lower and Upper bounds for range checking in HSV color space
    private Scalar mLowerBound = new Scalar(12, 100, 100);
    private Scalar mUpperBound = new Scalar(36, 255, 255);
    private List<MatOfPoint> mContours = new ArrayList<MatOfPoint>();

    private static final String TAG = "RingDetector";

    // Cache
    Mat mPyrDownMat = new Mat();
    Mat mHsvMat = new Mat();
    Mat mMask = new Mat();
    Mat mDilatedMask = new Mat();
    Mat mHierarchy = new Mat();


    public char process(Mat rgbaImage) throws InterruptedException {
        Imgproc.pyrDown(rgbaImage, mPyrDownMat);
        Imgproc.pyrDown(mPyrDownMat, mPyrDownMat);
        Imgproc.cvtColor(mPyrDownMat, mHsvMat, Imgproc.COLOR_RGB2HSV_FULL);
        Core.inRange(mHsvMat, mLowerBound, mUpperBound, mMask);
        Imgproc.dilate(mMask, mDilatedMask, new Mat());
        List<MatOfPoint> contours = new ArrayList<MatOfPoint>();
        Imgproc.findContours(mDilatedMask, contours, mHierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        // Find max contour area
        MatOfPoint maxContour = new MatOfPoint();
        double maxArea = 0;
        Iterator<MatOfPoint> each = contours.iterator();
        while (each.hasNext()) {
            MatOfPoint wrapper = each.next();
            double area = Imgproc.contourArea(wrapper);
            if (area > maxArea) {
                if (area > 1500) {
                    maxArea = area;
                    maxContour = wrapper;
                }
            }
        }

        // Filter contours by area and resize to fit the original image size
        mContours.clear();
        if (maxArea > 0) {
            Core.multiply(maxContour, new Scalar(4, 4), maxContour);
            mContours.add(maxContour);
            Rect rectBounding = Imgproc.boundingRect(maxContour);
            //Imgproc.rectangle(rgbaImage, new Point(rectBounding.x, rectBounding.y),
            //new Point(rectBounding.x + rectBounding.width, rectBounding.y + rectBounding.height)
            //new Scalar(33,137,255), 2);
            double ratio = (double) ((double)rectBounding.width /(double) rectBounding.height);
            //Log.d(TAG,  "Ratio =" + String.valueOf(ratio));
            if (ratio <= 2){
                return 'C';
            } else {
                return 'B';
            }
        } else
        {
            return 'A';
        }
    }
}