package org.firstinspires.ftc.teamcode;

import org.opencv.calib3d.Calib3d;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfDouble;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.MatOfPoint3f;
import org.opencv.core.Point;
import org.opencv.core.Point3;
import org.opencv.core.Scalar;
import org.opencv.utils.Converters;

import java.util.ArrayList;

import static org.opencv.core.Core.norm;
import static org.opencv.core.CvType.CV_64F;
import static org.opencv.core.CvType.CV_64FC1;
import static org.opencv.core.Scalar.all;

/**
 * Created by sovnik on 11/14/17.
 * God help me with this one
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

    public boolean intersectMollerTrumbore(Ray ray, Triangle triangle, double out) {
        final double epsilon = 0.000001;

        Point3 e1;
        Point3 e2;
        Point3 p;
        Point3 q;
        Point3 T;

        double det;
        double inv_det;
        double u;
        double v;
        double t2;

        Point3 v1 = triangle.getV0();
        Point3 v2 = triangle.getV1();
        Point3 v3 = triangle.getV2();

        Point3 o = ray.getP0();
        Point3 d = ray.getP1();

        e1 = sub(v2, v1);
        e2 = sub(v3, v1);
        p = cross(d, e2);
        det = dot(e1, p);
        if(det > -epsilon && det < epsilon) {
            return false;
        }
        inv_det = 1.f / det;
        T = sub(o, v1);
        u = dot(T, p) * inv_det;
        if(u < 0.f || u > 1.f) {
            return false;
        }
        q = cross(T, e1);
        v = dot(d, q);
        if(v < 0.f || u + v > 1.f) {
            return false;
        }
        t2 = dot(e2, q) * inv_det;

        if (t2 > epsilon) {
            out = t2;
            return true;
        }
        return  false;
    }

    public MatOfPoint2f verifyPoints (Mesh mesh) {
        MatOfPoint2f verifiedPoints2d = new MatOfPoint2f();

        for(int i = 0; i < mesh.getNumVertices(); i++) {
            Point3 point3d = mesh.getvertex(i);
            Point point2d = this.backproject3dPoint(point3d);
            verifiedPoints2d.toList().add(point2d);
        }

        return verifiedPoints2d;
    }

    public Point backproject3dPoint(Point3 point3d) {
        Mat point3dVec = new Mat();
        point3dVec.zeros(4, 1, CV_64FC1);

        point3dVec.put(0, 0, point3d.x);
        point3dVec.put(1, 0, point3d.y);
        point3dVec.put(2, 0, point3d.z);
        point3dVec.put(3, 0, 1);

        Mat point2dVec = new Mat();
        point2dVec.zeros(4, 1, CV_64FC1);

        Mat productAP = new Mat();

        Core.multiply(aMatrix, pMatrix, productAP);
        Core.multiply(productAP, point3dVec, point2dVec);

        Point point2d = new Point();
        //TODO: Check this
        double[] data = new double[2];
        point2d.x = point2dVec.get(0, 0, data) / point2dVec.get(2, 0, data);
        point2d.y = point2dVec.get(1, 0, data) / point2dVec.get(2, 0, data);

        return point2d;
    }

    public boolean backproject3dPoint(Mesh mesh, Point point2d, Point3 point3d) {
        ArrayList<ArrayList<Integer>> trianglesList = mesh.getTrianglesList();

        double lambda = 8;
        double u = point2d.x;
        double v = point2d.y;

        Mat point2dVec = new Mat();
        point2dVec.zeros(3, 1, CV_64F);

        point2dVec.put(0, 0, u * lambda);
        point2dVec.put(1, 0, v * lambda);
        point2dVec.put(2, 0, lambda);

        Mat xCamera = new Mat();
        Core.multiply(aMatrix.inv(), point2dVec, xCamera);

        Mat difference = new Mat();
        Mat xWorld = new Mat();
        Core.subtract(xCamera, tMatrix, difference);
        Core.multiply(rMatrix.inv(), difference, xWorld);

        Mat centerProjection = new Mat();
        Mat negativeInverse = new Mat();
        Core.multiply(rMatrix.inv(), Scalar.all(-1), negativeInverse);
        Core.multiply(negativeInverse, tMatrix, centerProjection);

        Mat ray = new Mat();
        Core.subtract(xWorld, centerProjection, ray);
        double rayNorm = Core.norm(ray);
        Core.divide(ray, Scalar.all(rayNorm), ray);
        //TODO: Check that also, who knows what it'll return
        ArrayList<Point3> centerProjectionPoints = new ArrayList<>();
        Converters.Mat_to_vector_Point3(centerProjection, centerProjectionPoints);
        ArrayList<Point3> rayPoints = new ArrayList<>();
        Converters.Mat_to_vector_Point3(centerProjection, rayPoints);
        Ray r = new Ray(centerProjectionPoints.get(0), centerProjectionPoints.get(0));
        ArrayList<Point3> intersectionsList = new ArrayList<>();

        for (int i = 0; i < trianglesList.size(); i++) {
            Point3 v0 = mesh.getvertex(trianglesList.get(i).get(0));
            Point3 v1 = mesh.getvertex(trianglesList.get(i).get(1));
            Point3 v2 = mesh.getvertex(trianglesList.get(i).get(2));

            Triangle t = new Triangle(i, v0, v1, v2);

            double out = 0;
            if(this.intersectMollerTrumbore(r, t, out)) {
                double tmpPX = (r.getP0().x) + (r.getP1().x * out);
                double tmpPY = (r.getP0().y) + (r.getP1().y * out);
                double tmpPZ = (r.getP0().z) + (r.getP1().z * out);
                Point3 tmpPoint = new Point3(tmpPX, tmpPY, tmpPZ);
                intersectionsList.add(tmpPoint);
            }
        }

        if(!intersectionsList.isEmpty()) {
            point3d = getNearest3dPoint(intersectionsList, r.getP0());
            return true;
        }

        else
        {
            return false;
        }
    }

    public boolean estimatePose(MatOfPoint3f listPoints3d, MatOfPoint2f listPoints2d, int flags) {
        MatOfDouble distCoefficients = new MatOfDouble();
        distCoefficients.zeros(4, 1, CV_64FC1);
        Mat rvec = new Mat();
        rvec.zeros(3, 1, CV_64FC1);
        Mat tvec = new Mat();
        tvec.zeros(3, 1, CV_64FC1);
        boolean useExtrinsicGuess = false;

        boolean correspondence = Calib3d.solvePnP(listPoints3d, listPoints2d, aMatrix, distCoefficients, rvec, tvec, useExtrinsicGuess, flags);

        Calib3d.Rodrigues(rvec, rMatrix);

        tMatrix = tvec;
        this.setPMatrix(rMatrix, tMatrix);

        return correspondence;
    }

    public void estimatePoseRANSAC(MatOfPoint3f listPoints3d, MatOfPoint2f listPoints2d, int flags, Mat inliers, int iterationsCount, float reprojectionError, double confidence) {
        MatOfDouble distCoefficients = new MatOfDouble();
        distCoefficients.zeros(4, 1, CV_64FC1);
        Mat rvec = new Mat();
        rvec.zeros(3, 1, CV_64FC1);
        Mat tvec = new Mat();
        tvec.zeros(3, 1, CV_64FC1);
        boolean useExtrinsicGuess = false;

        Calib3d.solvePnPRansac(listPoints3d, listPoints2d, aMatrix, distCoefficients, rvec, tvec, useExtrinsicGuess, iterationsCount, reprojectionError, confidence, inliers, flags);

        Calib3d.Rodrigues(rvec, rMatrix);
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
        pMatrix.put(0, 3, tMatrix.get(0, 0));
        pMatrix.put(1, 3, tMatrix.get(1, 0));
        pMatrix.put(3, 3, tMatrix.get(2, 0));

    }

    public Point3 getNearest3dPoint(ArrayList<Point3> pointsList, Point3 origin) {
        Point3 p1 = pointsList.get(0);
        Point3 p2 = pointsList.get(1);

        double d1 = Math.sqrt(Math.pow(p1.x-origin.x, 2) + Math.pow(p1.y-origin.y, 2) + Math.pow(p1.z-origin.z, 2));
        double d2 = Math.sqrt(Math.pow(p2.x-origin.x, 2) + Math.pow(p2.y-origin.y, 2) + Math.pow(p2.z-origin.z, 2));

        if(d1 > d2) {
            return p1;
        }
        else {
            return p2;
        }
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
        return v1.x * v2.x + v1.y * v2.y + v1.z * v2.z;
    }

    private Point3 sub(Point3 v1, Point3 v2) {
        Point3 tmpPoint = new Point3();
        tmpPoint.x = v1.x - v2.x;
        tmpPoint.y = v1.y - v2.y;
        tmpPoint.z = v1.z * v2.z;

        return tmpPoint;
    }
}
