/*
 * Derived from sample code provided by OpenCV in the OpenCV Android SDK:
 * https://github.com/opencv/opencv/releases/tag/3.2.0 
 */

package org.firstinspires.ftc.teamcode;

import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfInt;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;

public class ColorBlobDetector {
    // Lower and Upper bounds for range checking in HSV color space
    private Scalar mLowerBound = new Scalar(0);
    private Scalar mUpperBound = new Scalar(0);
    private Scalar mMinBound = new Scalar(0);
    private Scalar mMaxBound = new Scalar(0);
    private boolean hueOutOfRange;

    // Minimum contour area in percent for contours filtering
    private static double mMinContourArea = 0.1;

    // Color radius for range checking in HSV color space
    private Scalar mColorRadius = new Scalar(15, 50, 25, 0);// Adjust this as needed
    private Mat mSpectrum = new Mat();
    private List<MatOfPoint> mContours = new ArrayList<MatOfPoint>();

    //Outputs
    private Mat cvErodeOutput = new Mat();
    private Mat cvDilateOutput = new Mat();
    private Mat blurOutput = new Mat();
    private ArrayList<MatOfPoint> findContoursOutput = new ArrayList<MatOfPoint>();
    private Mat hsvThresholdOutput = new Mat();
    private ArrayList<MatOfPoint> filterContoursOutput = new ArrayList<MatOfPoint>();
    private ArrayList<MatOfPoint> convexHullsOutput = new ArrayList<MatOfPoint>();

    private Mat mMask = new Mat();
    private Mat mMask1 = new Mat();
    private Mat mMask2 = new Mat();

    public void setColorRadius(Scalar radius) {
        mColorRadius = radius;
    }

    public void setHsvColor(Scalar hsvColor) {
        double minH = hsvColor.val[0] - mColorRadius.val[0];
        double maxH = hsvColor.val[0] + mColorRadius.val[0];

        mLowerBound.val[0] = minH;
        mUpperBound.val[0] = maxH;

        hueOutOfRange = false;
        if (minH < 0) {
            hueOutOfRange = true;
            mLowerBound.val[0] += 256;
        } else if (maxH > 255) {
            hueOutOfRange = true;
            mUpperBound.val[0] -= 256;
        }

        mLowerBound.val[1] = hsvColor.val[1] - mColorRadius.val[1];
        mUpperBound.val[1] = hsvColor.val[1] + mColorRadius.val[1];

        mLowerBound.val[2] = hsvColor.val[2] - mColorRadius.val[2];
        mUpperBound.val[2] = hsvColor.val[2] + mColorRadius.val[2];

        mLowerBound.val[3] = 0;
        mUpperBound.val[3] = 255;

        mMinBound.val[0] = 0;
        mMinBound.val[1] = mLowerBound.val[1];
        mMinBound.val[2] = mLowerBound.val[2];
        mMinBound.val[3] = mLowerBound.val[3];

        mMaxBound.val[0] = 255;
        mMaxBound.val[1] = mUpperBound.val[1];
        mMaxBound.val[2] = mUpperBound.val[2];
        mMaxBound.val[3] = mUpperBound.val[3];

        Mat spectrumHsv = new Mat(1, (int) (maxH - minH), CvType.CV_8UC3);

        for (int j = 0; j < maxH - minH; j++) {
            int minHPlusJ = (int) (minH + j);
            if (minHPlusJ < 0) {
                minHPlusJ += 256;
            }
            byte[] tmp = {(byte) minHPlusJ, (byte) 255, (byte) 255};
            spectrumHsv.put(0, j, tmp);
        }

        Imgproc.cvtColor(spectrumHsv, mSpectrum, Imgproc.COLOR_HSV2RGB_FULL, 4);


    }

    public Mat getSpectrum() {
        return mSpectrum;
    }

    public void process(Mat rgbaImage) {
        Mat cvErodeSrc = rgbaImage;
        Mat cvErodeKernel = new Mat();
        Point cvErodeAnchor = new Point(-1, -1);
        double cvErodeIterations = 11.0;
        int cvErodeBordertype = Core.BORDER_CONSTANT;
        Scalar cvErodeBordervalue = new Scalar(-1);
        cvErode(cvErodeSrc, cvErodeKernel, cvErodeAnchor, cvErodeIterations, cvErodeBordertype, cvErodeBordervalue, cvErodeOutput);

        // Step CV_dilate0:
        Mat cvDilateSrc = cvErodeOutput;
        Mat cvDilateKernel = new Mat();
        Point cvDilateAnchor = new Point(-1, -1);
        double cvDilateIterations = 11.0;
        int cvDilateBordertype = Core.BORDER_CONSTANT;
        Scalar cvDilateBordervalue = new Scalar(-1);
        cvDilate(cvDilateSrc, cvDilateKernel, cvDilateAnchor, cvDilateIterations, cvDilateBordertype, cvDilateBordervalue, cvDilateOutput);

        // Step Blur0:
        Mat blurInput = cvDilateOutput;
        BlurType blurType = BlurType.get("Box Blur");
        double blurRadius = 7.2;
        blur(blurInput, blurType, blurRadius, blurOutput);

        if (hueOutOfRange) {
            Core.inRange(blurOutput, mMinBound, mUpperBound, mMask1);
            Core.inRange(blurOutput, mLowerBound, mMaxBound, mMask2);
            Core.bitwise_or(mMask2, mMask1, mMask);
        } else {
            Core.inRange(blurOutput, mLowerBound, mUpperBound, mMask);
        }

        // Step Filter_Contours0:
        ArrayList<MatOfPoint> filterContoursContours = findContoursOutput;
        double filterContoursMinArea = 0;
        double filterContoursMinPerimeter = 0;
        double filterContoursMinWidth = 0;
        double filterContoursMaxWidth = 10000.0;
        double filterContoursMinHeight = 0;
        double filterContoursMaxHeight = 5000.0;
        double[] filterContoursSolidity = {80.93525179856115, 100.0};
        double filterContoursMaxVertices = 1000000;
        double filterContoursMinVertices = 0;
        double filterContoursMinRatio = 0;
        double filterContoursMaxRatio = 1000;
        filterContours(filterContoursContours, filterContoursMinArea, filterContoursMinPerimeter, filterContoursMinWidth, filterContoursMaxWidth, filterContoursMinHeight, filterContoursMaxHeight, filterContoursSolidity, filterContoursMaxVertices, filterContoursMinVertices, filterContoursMinRatio, filterContoursMaxRatio, filterContoursOutput);


        List<MatOfPoint> contours = new ArrayList<>();

        Iterator<MatOfPoint> each = contours.iterator();

        ArrayList<MatOfPoint> convexHullsContours = filterContoursOutput;
        convexHulls(convexHullsContours, convexHullsOutput);
    }



    public List<MatOfPoint> getContours() {
        return mContours;
    }

    public Mat cvErodeOutput() {
        return cvErodeOutput;
    }
    public Mat cvDilateOutput() {
        return cvDilateOutput;
    }
    public Mat blurOutput() {
        return blurOutput;
    }
    public ArrayList<MatOfPoint> findContoursOutput() {
        return findContoursOutput;
    }
    public Mat hsvThresholdOutput() {
        return hsvThresholdOutput;
    }
    public ArrayList<MatOfPoint> filterContoursOutput() {
        return filterContoursOutput;
    }
    public ArrayList<MatOfPoint> convexHullsOutput() {
        return convexHullsOutput;
    }

    private void cvErode(Mat src, Mat kernel, Point anchor, double iterations,
                         int borderType, Scalar borderValue, Mat dst) {
        if (kernel == null) {
            kernel = new Mat();
        }
        if (anchor == null) {
            anchor = new Point(-1,-1);
        }
        if (borderValue == null) {
            borderValue = new Scalar(-1);
        }
        Imgproc.erode(src, dst, kernel, anchor, (int)iterations, borderType, borderValue);
    }

    private void cvDilate(Mat src, Mat kernel, Point anchor, double iterations,
                          int borderType, Scalar borderValue, Mat dst) {
        if (kernel == null) {
            kernel = new Mat();
        }
        if (anchor == null) {
            anchor = new Point(-1,-1);
        }
        if (borderValue == null){
            borderValue = new Scalar(-1);
        }
        Imgproc.dilate(src, dst, kernel, anchor, (int)iterations, borderType, borderValue);
    }

    enum BlurType{
        BOX("Box Blur"), GAUSSIAN("Gaussian Blur"), MEDIAN("Median Filter"),
        BILATERAL("Bilateral Filter");

        private final String label;

        BlurType(String label) {
            this.label = label;
        }

        public static BlurType get(String type) {
            if (BILATERAL.label.equals(type)) {
                return BILATERAL;
            }
            else if (GAUSSIAN.label.equals(type)) {
                return GAUSSIAN;
            }
            else if (MEDIAN.label.equals(type)) {
                return MEDIAN;
            }
            else {
                return BOX;
            }
        }

        @Override
        public String toString() {
            return this.label;
        }
    }

    private void blur(Mat input, BlurType type, double doubleRadius,
                      Mat output) {
        int radius = (int)(doubleRadius + 0.5);
        int kernelSize;
        switch(type){
            case BOX:
                kernelSize = 2 * radius + 1;
                Imgproc.blur(input, output, new Size(kernelSize, kernelSize));
                break;
            case GAUSSIAN:
                kernelSize = 6 * radius + 1;
                Imgproc.GaussianBlur(input,output, new Size(kernelSize, kernelSize), radius);
                break;
            case MEDIAN:
                kernelSize = 2 * radius + 1;
                Imgproc.medianBlur(input, output, kernelSize);
                break;
            case BILATERAL:
                Imgproc.bilateralFilter(input, output, -1, radius, radius);
                break;
        }
    }

    private void findContours(Mat input, boolean externalOnly,
                              List<MatOfPoint> contours) {
        Mat hierarchy = new Mat();
        contours.clear();
        int mode;
        if (externalOnly) {
            mode = Imgproc.RETR_EXTERNAL;
        }
        else {
            mode = Imgproc.RETR_LIST;
        }
        int method = Imgproc.CHAIN_APPROX_SIMPLE;
        Imgproc.findContours(input, contours, hierarchy, mode, method);
    }

    private void filterContours(List<MatOfPoint> inputContours, double minArea,
                                double minPerimeter, double minWidth, double maxWidth, double minHeight, double
                                        maxHeight, double[] solidity, double maxVertexCount, double minVertexCount, double
                                        minRatio, double maxRatio, List<MatOfPoint> output) {
        final MatOfInt hull = new MatOfInt();
        output.clear();
        //operation
        for (int i = 0; i < inputContours.size(); i++) {
            final MatOfPoint contour = inputContours.get(i);
            final Rect bb = Imgproc.boundingRect(contour);
            if (bb.width < minWidth || bb.width > maxWidth) continue;
            if (bb.height < minHeight || bb.height > maxHeight) continue;
            final double area = Imgproc.contourArea(contour);
            if (area < minArea) continue;
            if (Imgproc.arcLength(new MatOfPoint2f(contour.toArray()), true) < minPerimeter) continue;
            Imgproc.convexHull(contour, hull);
            MatOfPoint mopHull = new MatOfPoint();
            mopHull.create((int) hull.size().height, 1, CvType.CV_32SC2);
            for (int j = 0; j < hull.size().height; j++) {
                int index = (int)hull.get(j, 0)[0];
                double[] point = new double[] { contour.get(index, 0)[0], contour.get(index, 0)[1]};
                mopHull.put(j, 0, point);
            }
            final double solid = 100 * area / Imgproc.contourArea(mopHull);
            if (solid < solidity[0] || solid > solidity[1]) continue;
            if (contour.rows() < minVertexCount || contour.rows() > maxVertexCount)	continue;
            final double ratio = bb.width / (double)bb.height;
            if (ratio < minRatio || ratio > maxRatio) continue;
            output.add(contour);
        }
    }

    /**
     * Compute the convex hulls of contours.
     * @param inputContours The contours on which to perform the operation.
     * @param outputContours The contours where the output will be stored.
     */
    private void convexHulls(List<MatOfPoint> inputContours,
                             ArrayList<MatOfPoint> outputContours) {
        final MatOfInt hull = new MatOfInt();
        outputContours.clear();
        for (int i = 0; i < inputContours.size(); i++) {
            final MatOfPoint contour = inputContours.get(i);
            final MatOfPoint mopHull = new MatOfPoint();
            Imgproc.convexHull(contour, hull);
            mopHull.create((int) hull.size().height, 1, CvType.CV_32SC2);
            for (int j = 0; j < hull.size().height; j++) {
                int index = (int) hull.get(j, 0)[0];
                double[] point = new double[] {contour.get(index, 0)[0], contour.get(index, 0)[1]};
                mopHull.put(j, 0, point);
            }
            outputContours.add(mopHull);
        }
    }
}