package org.firstinspires.ftc.teamcode;

/**
 * Created by sovnik on 11/11/17.
 * VERY VERY EXPERIMENTAL DO NOT USE THIS FOR ANYTHING. It's harmless if you don't bother it.
 */

import com.qualcomm.robotcore.util.Util;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfDMatch;
import org.opencv.core.MatOfKeyPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.MatOfPoint3f;
import org.opencv.core.Point;
import org.opencv.core.Point3;
import org.opencv.core.Scalar;
import org.opencv.features2d.DescriptorExtractor;
import org.opencv.features2d.DescriptorMatcher;
import org.opencv.features2d.FeatureDetector;
import org.opencv.features2d.FlannBasedMatcher;
import org.opencv.imgproc.*;
import org.opencv.calib3d.*;
import org.opencv.video.KalmanFilter;
import org.opencv.video.Video;
import org.firstinspires.ftc.teamcode.poseDetection.*;

import java.util.ArrayList;

import static org.opencv.calib3d.Calib3d.SOLVEPNP_ITERATIVE;
import static org.opencv.core.CvType.CV_64F;

public class GlyphPoseDetector {

    double f = 55; // Focal length
    double sensorX = 22.3; // Sensor size
    double sensorY = 14.9;
    double width = 1280; // Image size
    double height = 720;

    double paramsCamera[] = new double[] {
            width * f / sensorX,
            height * f / sensorY,
            width / 2,
            height / 2
    };

    Scalar red = new Scalar(0, 0, 255);
    Scalar green = new Scalar(0, 255, 0);
    Scalar blue = new Scalar(255, 0, 0);
    Scalar yellow = new Scalar(0, 255, 255);

    int numKeyPoints = 2000;
    float ratioTest = 0.70f;
    boolean fastMatch = true;

    int iterationsCount = 500;
    float reprojectionError = 2.0f;
    double confidence = 0.95;

    int minInliersKalman = 30;

    KalmanFilter kf;
    int nStates = 18;
    int nMeasurements = 6;
    int nInputs = 0;
    double dt = 0.125;

    int pnpMethod = SOLVEPNP_ITERATIVE;

    PnPProblem pnpDetection = new PnPProblem(paramsCamera);
    PnPProblem pnpDetectionEst = new PnPProblem(paramsCamera);
    RobustMatcher rMatcher;
    FeatureDetector orb = FeatureDetector.create(FeatureDetector.ORB);
    Model model;
    Mesh mesh;
    Mat measurements = new Mat(nMeasurements, 1, CV_64F);
    Boolean goodMeasurement = false;

    Mat outputFrame;

    ArrayList<Point3> listPoints3dModel;
    Mat descriptorsModel;

    public void initKalmanFilter(KalmanFilter kf, int nStates, int nMeasurements, int nInputs, double dt) {
        kf = new KalmanFilter(nStates, nMeasurements, nInputs, CV_64F);
        Mat processNoise = new Mat();
        Mat measureNoise = new Mat();
        Mat errorCovariance = new Mat();
        processNoise.setTo(Scalar.all(1e-5));
        measureNoise.setTo(Scalar.all(1e-2));
        errorCovariance.setTo(Scalar.all(1));
        kf.set_processNoiseCov(processNoise);
        kf.set_measurementNoiseCov(measureNoise);
        kf.set_errorCovPost(errorCovariance);

        kf.get_transitionMatrix().put(0, 3, dt);
        kf.get_transitionMatrix().put(1, 4, dt);
        kf.get_transitionMatrix().put(2, 5, dt);
        kf.get_transitionMatrix().put(3, 6, dt);
        kf.get_transitionMatrix().put(4, 7, dt);
        kf.get_transitionMatrix().put(5, 8, dt);
        kf.get_transitionMatrix().put(0, 6, (0.5 * Math.pow(dt, 2)));
        kf.get_transitionMatrix().put(1, 7, (0.5 * Math.pow(dt, 2)));
        kf.get_transitionMatrix().put(2, 8, (0.5 * Math.pow(dt, 2)));

        kf.get_transitionMatrix().put(9, 12, dt);
        kf.get_transitionMatrix().put(10, 13, dt);
        kf.get_transitionMatrix().put(11, 14, dt);
        kf.get_transitionMatrix().put(12, 15, dt);
        kf.get_transitionMatrix().put(13, 16, dt);
        kf.get_transitionMatrix().put(14, 17, dt);
        kf.get_transitionMatrix().put(9, 15, (0.5 * Math.pow(dt, 2)));
        kf.get_transitionMatrix().put(10, 16, (0.5 * Math.pow(dt, 2)));
        kf.get_transitionMatrix().put(11, 17, (0.5 * Math.pow(dt, 2)));

        kf.get_measurementMatrix().put(0, 0, 1);
        kf.get_measurementMatrix().put(1, 1, 1);
        kf.get_measurementMatrix().put(2, 2, 1);
        kf.get_measurementMatrix().put(3, 9, 1);
        kf.get_measurementMatrix().put(4, 10, 1);
        kf.get_measurementMatrix().put(5, 11, 1);
    }

    public void updateKalmanFilter(KalmanFilter kf, Mat measurements, Mat translationEst, Mat rotationEst) {
        Mat prediction = kf.predict();
        Mat estimated = kf.correct(measurements);

        double[] data = new double[2];
        translationEst.put(0, 0, estimated.get(0, 0, data));
        translationEst.put(1, 0, estimated.get(1, 0, data));
        translationEst.put(2, 0, estimated.get(2, 0, data));

        Mat eulersEstimated = new Mat();
        eulersEstimated.zeros(3, 1, CV_64F);
        eulersEstimated.put(0, 0, estimated.get(9, 0, data));
        eulersEstimated.put(1, 0, estimated.get(10, 0, data));
        eulersEstimated.put(2, 0, estimated.get(11, 0, data));

        rotationEst = Utils.eulerToRot(eulersEstimated);
    }

    public void fillMeasurements(Mat measurements, Mat translationMeasured, Mat rotationMeasured) {
        Mat measuredEulers = new Mat();
        measuredEulers.zeros(3, 1, CV_64F);

        double[] data = new double[5];
        measurements.put(0, 0, translationMeasured.get(0, 0, data));
        measurements.put(1, 0, translationMeasured.get(1, 0, data));
        measurements.put(2, 0, translationMeasured.get(2, 0, data));

        measurements.put(3, 0, measuredEulers.get(0, 0, data));
        measurements.put(4, 0, measuredEulers.get(1, 0, data));
        measurements.put(5, 0, measuredEulers.get(2, 0, data));
    }

    public Mat processFrame(Mat frame) {
        MatOfDMatch goodMatches = new MatOfDMatch();
        MatOfKeyPoint keypointsScene = new MatOfKeyPoint();

        if(fastMatch) {
            rMatcher.fastRobustMatch(frame, goodMatches, keypointsScene, descriptorsModel);
        }
        else {
            rMatcher.robustMatch(frame, goodMatches, keypointsScene, descriptorsModel);
        }

        MatOfPoint3f listPoints3dModelMatch = new MatOfPoint3f();
        MatOfPoint2f listPoints2dSceneMatch = new MatOfPoint2f();

        for(int matchIndex = 0; matchIndex < goodMatches.toList().size(); matchIndex++) {
            Point3 point3dModel = listPoints3dModel.get(goodMatches.toList().get(matchIndex).trainIdx);
            Point point2dScene = keypointsScene.toList().get(goodMatches.toList().get(matchIndex).queryIdx).pt;
            listPoints3dModelMatch.toList().add(point3dModel);
            listPoints2dSceneMatch.toList().add(point2dScene);
        }

        Utils.draw2dPoints(outputFrame, listPoints2dSceneMatch.toList(), red);

        Mat inliersIdx = new Mat();
        int[] data = new int[0];

        MatOfPoint2f listPoints2dInliers = new MatOfPoint2f();

        if(goodMatches.toList().size() > 0) {
            pnpDetection.estimatePoseRANSAC(listPoints3dModelMatch, listPoints2dSceneMatch, pnpMethod, inliersIdx, iterationsCount, reprojectionError, confidence);

            for (int inliersIndex = 0; inliersIndex < inliersIdx.rows(); inliersIndex++) {
                int n = inliersIdx.get(inliersIndex, 0, data);
                Point point2d = listPoints2dSceneMatch.toList().get(n);
                listPoints2dInliers.toList().add(point2d);
            }

            Utils.draw2dPoints(outputFrame, listPoints2dInliers.toList(), blue);

            goodMeasurement = false;

            if(inliersIdx.rows() >= minInliersKalman) {
                Mat translationMeasured = new Mat(3, 1, CV_64F);
                translationMeasured = pnpDetection.getTMatrix();

                Mat rotationMeasured = new Mat(3, 3, CV_64F);
                rotationMeasured = pnpDetection.getRMatrix();

                fillMeasurements(measurements, translationMeasured, rotationMeasured);

                goodMeasurement = true;
            }

            Mat translationEstimate = new Mat(3, 1, CV_64F);
            Mat rotationEstimate = new Mat(3, 3, CV_64F);

            updateKalmanFilter(kf, measurements, translationEstimate, rotationEstimate);
            pnpDetectionEst.setPMatrix(rotationEstimate, translationEstimate);

            if(goodMeasurement) {
                Utils.drawObjectMesh(outputFrame, mesh, pnpDetection, green);
            }
            else {
                Utils.drawObjectMesh(outputFrame, mesh, pnpDetectionEst, yellow);
            }

            float l = 5;

            ArrayList<Point> posePoints2d = new ArrayList<>();

            posePoints2d.add(pnpDetectionEst.backproject3dPoint(new Point3(0, 0, 0)));
            posePoints2d.add(pnpDetectionEst.backproject3dPoint(new Point3(l, 0, 0)));
            posePoints2d.add(pnpDetectionEst.backproject3dPoint(new Point3(0, l, 0)));
            posePoints2d.add(pnpDetectionEst.backproject3dPoint(new Point3(0, 0, l)));
            Utils.draw3dCoordinateAxes(outputFrame, posePoints2d);

            double detectionRatio = ((double)inliersIdx.rows()/(double)goodMatches.toList().size()) * 100;
            Utils.drawConfidence(outputFrame, detectionRatio, yellow);

            int inliersInt = inliersIdx.rows();
            int outliersInt = goodMatches.toList().size() - inliersInt;
            String inliersStr = Integer.toString(inliersInt);
            String outliersStr = Integer.toString(outliersInt);
            String n = Integer.toString(goodMatches.toList().size());

            String text = "Found " + inliersStr + " of " + n + " matches";
            String text2 = "Inliers: " + inliersStr + " - Outliers: " + outliersStr;

            Utils.drawText(outputFrame, text, green);
            Utils.drawText(outputFrame, text2, red);

        }

        return outputFrame;
    }

    public void initializePoseDetection(String meshPath, String modelPath, String paramPath) throws java.io.FileNotFoundException {
        mesh.load(meshPath);
        model.load(modelPath);
        rMatcher.setFeatureDetector(orb);
        rMatcher.setDescriptorExtractor(new DescriptorExtractor(DescriptorExtractor.ORB));
        DescriptorMatcher matcher = DescriptorMatcher.create(DescriptorMatcher.FLANNBASED);
        matcher.read(paramPath);
        rMatcher.setDescriptorMatcher(matcher);
        initKalmanFilter(kf, nStates, nMeasurements, nInputs, dt);
        measurements.setTo(new Scalar(0));
        listPoints3dModel = model.getPoints3d();
        descriptorsModel = model.getDescriptors();

    }
}
