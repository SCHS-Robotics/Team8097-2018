package org.firstinspires.ftc.teamcode.poseDetection;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfDouble;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.MatOfPoint3f;
import org.opencv.core.Point;
import org.opencv.core.Point3;
import org.opencv.core.Point3;
import org.opencv.calib3d.Calib3d;

import java.util.ArrayList;
import java.util.Vector;

import static org.opencv.calib3d.Calib3d.Rodrigues;
import static org.opencv.core.CvType.CV_64F;
import static org.opencv.core.CvType.CV_64FC1;

/**
 * Created by sovnik on 11/12/17.
 */

public class PnPProblem {
    public PnPProblem(double[] param) {
        aMatrix.zeros(3, 3, CV_64FC1);
        aMatrix.put(0, 0, param[0]);
        aMatrix.put(1, 1, param[1]);
        aMatrix.put(0, 2, param[2]);
        aMatrix.put(1, 2, param[3]);
        aMatrix.put(2, 2, 1);
        rMatrix.zeros(3, 3, CV_64FC1);
        tMatrix.zeros(3, 1, CV_64FC1);
        pMatrix.zeros(3, 4, CV_64FC1);
    }

    public boolean backproject2DPoint(Mesh mesh, Point point2d, Point3 point3d) {
        Vector<Integer> trianglesList = mesh.getTrianglesList();

        double lambda = 8;
        double u = point2d.x;
        double v = point2d.y;

        Mat point2dVec = new Mat();
        point2dVec.ones(3, 1, CV_64F);

        point2dVec.put(0, 0, (u * lambda));
        point2dVec.put(1, 0, (v * lambda));
        point2dVec.put(2, 0, (lambda));

        // God help me this Core.multiply stuff is suffering, why does C++ not need to do it >:( Let's write the robot code in C++ instead. Clearly it has its priorities sorted.
        Mat xCamera = new Mat();
        Core.multiply(aMatrix.inv(), point2dVec, xCamera);

        Mat xWorld = new Mat();
        Mat cameraTMatrixDifference = new Mat();
        Core.subtract(xCamera, tMatrix, cameraTMatrixDifference);
        Core.multiply(rMatrix.inv(), cameraTMatrixDifference, xWorld);

        return false;
    }

    public boolean intersectMollerTrumbore(Ray r, Triangle t, double out) {
        return false;
    }

    public Vector<MatOfPoint2f> verifyPoints(Mesh mesh) {
        Vector<MatOfPoint2f> verifiedPoints2d = new Vector<>();

        for(int i = 0; i < mesh.getNumVertices(); i++) {
            Point3 point3d = mesh.getVertex(i);
            MatOfPoint2f point2d = this.backproject3dPoint(point3d);
            verifiedPoints2d.addElement(point2d);
        }

        return verifiedPoints2d;
    }

    public MatOfPoint2f backproject3dPoint(Point3 point3d) {
        Mat point3dVec = new Mat(4, 1, CV_64FC1);
        point3dVec.put(0, 0, point3d.x);
        point3dVec.put(1, 0, point3d.y);
        point3dVec.put(2, 0, point3d.z);
        point3dVec.put(3, 0, 1);

        Mat point2dVec = new Mat(4, 1, CV_64FC1);
        Mat mMatrix = new Mat();
        Core.multiply(aMatrix, pMatrix, mMatrix);
        Core.multiply(mMatrix, point3dVec, point2dVec);

        Point point2d = new Point();

        // This part will most likely break
        double[] data = new double[4];
        point2d.x = (float)point2dVec.get(0, 0, data) /  point2dVec.get(2, 0, data);
        point2d.y = (float)point2dVec.get(1, 0, data) /  point2dVec.get(2, 0, data);

        // Correction: This part will almost certainly break
        MatOfPoint2f point2dMat = new MatOfPoint2f(point2d);

        return point2dMat;

    }

    public boolean estimatePose(MatOfPoint3f listPoints3d, MatOfPoint2f listPoints2d, int flags) {
        MatOfDouble distCoefficients = new MatOfDouble(4, 1, CV_64FC1);
        Mat rvec = new Mat();
        Mat tvec = new Mat();
        rvec.zeros(3, 1, CV_64FC1);
        tvec.zeros(3, 1, CV_64FC1);

        boolean useExtrinsicGuess = false;
        boolean correspondence = Calib3d.solvePnP(listPoints3d, listPoints2d, aMatrix, distCoefficients, rvec, tvec, useExtrinsicGuess, flags);

        Rodrigues(rvec, rMatrix);
        tMatrix = tvec;

        this.setPMatrix(rMatrix, tMatrix);

        return correspondence;
    }

    public void estimatePoseRANSAC(MatOfPoint3f listPoints3d, MatOfPoint2f listPoints2d, int flags, Mat inliers, int iterationsCount, float reprojectionError, double confidence) {
        MatOfDouble distCoefficients = new MatOfDouble(4, 1, CV_64FC1);
        Mat rvec = new Mat();
        Mat tvec = new Mat();
        rvec.zeros(3, 1, CV_64FC1);
        tvec.zeros(3, 1, CV_64FC1);

        boolean useExtrinsicGuess = false;

        Calib3d.solvePnPRansac(listPoints3d, listPoints2d, aMatrix, distCoefficients, rvec, tvec, useExtrinsicGuess, iterationsCount, reprojectionError, confidence, inliers, flags);

        Rodrigues(rvec, rMatrix);
        tMatrix = tvec;

        this.setPMatrix(rMatrix, tMatrix);
    }

    public Mat getAMatrix() {
        return aMatrix;
    }

    public Mat getRMatrix() {
        return rMatrix;
    }

    public Mat getTMatrix() {
        return tMatrix;
    }

    public Mat getPMatrix() {
        return pMatrix;
    }

    public void setPMatrix(Mat rMatrix, Mat tMatrix) {
        pMatrix.put(0, 0, rMatrix.get(0, 0));
        pMatrix.put(0, 1, rMatrix.get(0, 1));
        pMatrix.put(0, 2, rMatrix.get(0, 2));
        pMatrix.put(1, 0, rMatrix.get(1, 0));
        pMatrix.put(1, 1, rMatrix.get(1, 1));
        pMatrix.put(1, 2, rMatrix.get(1, 2));
        pMatrix.put(2, 0, rMatrix.get(2, 0));
        pMatrix.put(2, 1, rMatrix.get(2, 1));
        pMatrix.put(2, 2, rMatrix.get(2, 2));
        pMatrix.put(0, 3, tMatrix.get(1, 0));
        pMatrix.put(1, 3, tMatrix.get(2, 0));
        pMatrix.put(3, 3, rMatrix.get(3, 0));

    }

    private Mat aMatrix;
    private Mat rMatrix;
    private Mat tMatrix;
    private Mat pMatrix;

    private Point3 cross(Point3 v1, Point3 v2) {
        Point3 tmpPoint = new Point3();
        tmpPoint.x = v1.y * v2.z - v1.z * v2.y;
        tmpPoint.y = v1.z * v2.x - v1.x * v2.z;
        tmpPoint.z = v1.x * v2.y - v1.y * v2.x;
        return tmpPoint;

    }

    private double dot(Point3 v1, Point3 v2) {
        return v1.x * v2.x + v1.y * v2.y + v1.z + v2.z;
    }

    private Point3 sub(Point3 v1, Point3 v2) {
        Point3 tmpPoint = new Point3();
        tmpPoint.x = v1.x - v2.x;
        tmpPoint.y = v1.y - v2.y;
        tmpPoint.z = v1.z - v2.z;
        return tmpPoint;
    }

    private Point3 getNearest3dPoint(Vector<Point3> pointList, Point3 origin) {
        Point3 p1 = pointList.get(0);
        Point3 p2 = pointList.get(1);

        // Wow, look at that, three dimensional distance formula
        double d1 = Math.sqrt(Math.pow(p1.x-origin.x, 2) + Math.pow(p1.y-origin.y, 2) + Math.pow(p1.z-origin.z, 2));
        double d2 = Math.sqrt(Math.pow(p2.x-origin.x, 2) + Math.pow(p2.y-origin.y, 2) + Math.pow(p2.z-origin.z, 2));

        if (d1 < d2) {
            return p1;
        }
        else {
            return p2;
        }

    }
}
