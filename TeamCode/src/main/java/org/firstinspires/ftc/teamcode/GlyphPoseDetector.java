package org.firstinspires.ftc.teamcode;

/**
 * Created by sovnik on 11/11/17.
 * VERY VERY EXPERIMENTAL DO NOT USE THIS FOR ANYTHING. It's harmless if you don't bother it.
 */

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.imgproc.*;
import org.opencv.calib3d.*;
import org.opencv.video.KalmanFilter;
import org.opencv.video.Video;

import static org.opencv.calib3d.Calib3d.SOLVEPNP_ITERATIVE;

public class GlyphPoseDetector {

    double f = 55; // Focal length
    double sensorX = 22.3; // Sensor size
    double sensorY = 14.9;
    double width = 1280; // Image size
    double height = 720;

    double cameraParams[] = {
            width * f / sensorX,
            height * f / sensorY,
            width / 2,
            height / 2
    };

    // Colors
    Scalar red = new Scalar(0, 0, 255);
    Scalar green = new Scalar(0, 255, 0);
    Scalar blue = new Scalar(255, 0, 0);
    Scalar yellow = new Scalar(0, 0, 255);

    int numKeyPoints = 2700;
    float ratioTest = 0.70f;
    boolean fast_match = true;

    int iterationsCount = 500;
    float reprojectionError = 2.0f;
    double confidence = 0.95;

    int minInliersKalman = 30;

    int pnpMethod = SOLVEPNP_ITERATIVE;

    public void initKalmanFilter (KalmanFilter kf, int nStates, int nMeasurements, int nInputs, double dt) {

    }

    public void updateKalmanFilter (KalmanFilter kf, Mat measurements, Mat translationEstimated, Mat rotationEstimated){

    }

    public void fillMeasurements (Mat measurements, final Mat translationMeasured, final Mat rotationMeasured){

    }

}
