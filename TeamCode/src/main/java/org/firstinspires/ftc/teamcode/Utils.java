package org.firstinspires.ftc.teamcode;

import org.opencv.calib3d.Calib3d;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Point3;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;

import static org.opencv.core.CvType.CV_64F;

/**
 * Created by sovnik on 11/15/17.
 */

public class Utils {

    public void drawText(Mat image, String text, Scalar color) {
        Imgproc.putText(image, text, new Point(25, 50), fontFace, fontScale, color, fontThickness);
    }

    public void drawFPS(Mat image, double fps, Scalar color) {
        String fpsStr = Integer.toString((int) fps);
        String text = fpsStr + " FPS";
        Imgproc.putText(image, text, new Point(500, 50), fontFace, fontScale, color, fontThickness);
    }

    public void drawConfidence(Mat image, double confidence, Scalar color) {
        String confStr = Integer.toString((int) confidence);
        String text = confStr + "%";
        Imgproc.putText(image, text, new Point(500, 50), fontFace, fontScale, color, fontThickness);
    }

    public void drawCounter(Mat image, int n, int nMax, Scalar color) {
        String nStr = Integer.toString((int) n);
        String nMaxStr = Integer.toString((int) nMax);
        String text = nStr + " of " + nMaxStr + " points";
        Imgproc.putText(image, text, new Point(500, 50), fontFace, fontScale, color, fontThickness);
    }

    public void drawPoints(Mat image, ArrayList<Point> listPoints2d, ArrayList<Point3> listPoints3d, Scalar color) {
        for(int i = 0; i < listPoints2d.size(); i++) {
            Point point2d = listPoints2d.get(i);
            Point3 point3d = listPoints3d.get(i);

            Imgproc.circle(image, point2d, radius, color, -1);

            String idx = Integer.toString(i+1);
            String x = Integer.toString((int) point3d.x);
            String y = Integer.toString((int) point3d.y);
            String z = Integer.toString((int) point3d.z);

            String text = "P" + idx + " (" + x + "," + y + "," + z + ")";

            point2d.x = point2d.x + 10;
            point2d.y = point2d.y - 10;

            Imgproc.putText(image, text, point2d, fontFace, fontScale*0.5, color, fontThickness);

        }
    }

    public void draw2dPoints(Mat image, ArrayList<Point> listPoints, Scalar color) {
        for(int i = 0; i < listPoints.size(); i++) {
            Point point2d = listPoints.get(i);
            Imgproc.circle(image, point2d, radius, color, -1);
        }
    }

    public void drawArrow(Mat image, Point p, Point q, Scalar color, int arrowMagnitude, int thickness, int lineType, int shift) {
        Imgproc.line(image, p, q, color, thickness, lineType, shift);
        double angle = Math.atan2((double)p.y - q.y, (double)p.x - q.x);

        p.x = (int) (q.x + arrowMagnitude * Math.cos(angle + Math.PI / 4));
        p.y = (int) (q.y + arrowMagnitude * Math.sin(angle + Math.PI / 4));
        Imgproc.line(image, p, q, color, thickness, lineType, shift);
        p.x = (int) (q.x + arrowMagnitude * Math.cos(angle - Math.PI / 4));
        p.y = (int) (q.y + arrowMagnitude * Math.sin(angle - Math.PI / 4));
        Imgproc.line(image, p, q, color, thickness, lineType, shift);
    }

    public void draw3dCoordinateAxes(Mat image, ArrayList<Point> listPoints2d) {
        Scalar red = new Scalar(0, 0, 255);
        Scalar green = new Scalar(0, 255, 0);
        Scalar blue = new Scalar(255, 0, 0);
        Scalar black = new Scalar(0, 0, 0);

        Point origin = listPoints2d.get(0);
        Point pointX = listPoints2d.get(1);
        Point pointY = listPoints2d.get(2);
        Point pointZ = listPoints2d.get(3);

        Imgproc.arrowedLine(image, origin, pointX, red, 9, lineType, 2, 3);
        Imgproc.arrowedLine(image, origin, pointY, blue, 9, lineType, 2, 3);
        Imgproc.arrowedLine(image, origin, pointZ, green, 9, lineType, 2, 3);
        Imgproc.circle(image, origin, radius / 2, black, -1, lineType, 0);
    }

    public void drawObjectMesh(Mat image, Mesh mesh, PnPProblem pnPProblem, Scalar color) {
        ArrayList<ArrayList<Integer>> listTriangles = mesh.getTrianglesList();
        for(int i = 0; i < listTriangles.size(); i++) {
            ArrayList<Integer> tmpTriangle = listTriangles.get(i);

            Point3 point3d0 = mesh.getvertex(tmpTriangle.get(0));
            Point3 point3d1 = mesh.getvertex(tmpTriangle.get(1));
            Point3 point3d2 = mesh.getvertex(tmpTriangle.get(2));

            Point point2d0 = pnPProblem.backproject3dPoint(point3d0);
            Point point2d1 = pnPProblem.backproject3dPoint(point3d1);
            Point point2d2 = pnPProblem.backproject3dPoint(point3d2);

            Imgproc.line(image, point2d0, point2d1, color, 1);
            Imgproc.line(image, point2d1, point2d2, color, 1);
            Imgproc.line(image, point2d2, point2d0, color, 1);
        }
    }

    public double getTranslationError(Mat tTrue, Mat t) {
        Mat difference = new Mat();
        Core.subtract(tTrue, t, difference);
        return Core.norm(difference);
    }

    public double getRotationError(Mat rTrue, Mat r) {
        Mat errorVec = new Mat();
        Mat errorMat = new Mat();
        Mat negativeInverse = new Mat();
        Core.multiply(r.inv(), Scalar.all(-1), negativeInverse);
        Core.multiply(rTrue, negativeInverse, errorMat);

        Calib3d.Rodrigues(errorMat, errorVec);

        return Core.norm(errorVec);
    }

    public Mat rotToEuler(Mat rotationMatrix) {
        Mat euler = new Mat();
        euler.zeros(3, 1, CV_64F);
        double[] data = new double[3];
        double m00 = rotationMatrix.get(0, 0, data);
        double m02 = rotationMatrix.get(0, 2, data);
        double m10 = rotationMatrix.get(1, 0, data);
        double m11 = rotationMatrix.get(1, 1, data);
        double m12 = rotationMatrix.get(1, 2, data);
        double m20 = rotationMatrix.get(2, 0, data);
        double m22 = rotationMatrix.get(2, 2, data);

        double x;
        double y;
        double z;

        if(m10 > 0.998) {
            x = 0;
            y = Math.PI / 2;
            z = Math.atan2(m02, m22);
        }
        else if(m10 < -0.998) {
            x = 0;
            y = -Math.PI / 2;
            z = Math.atan2(m02, m22);
        }
        else {
            x = Math.atan2(-m12, m11);
            y = -Math.PI / 2;
            z = Math.atan2(m02, m22);
        }

        euler.put(0, 0, x);
        euler.put(1, 0, y);
        euler.put(2, 0, z);

        return euler;
    }

    public Mat eulerToRot(Mat euler) {
        Mat rotationMatrix = new Mat();
        rotationMatrix.zeros(3,  3, CV_64F);
        double[] data = new double[3];
        double x = euler.get(0, 0, data);
        double y = euler.get(1, 0, data);
        double z = euler.get(2, 0, data);

        double ch = Math.cos(z);
        double sh = Math.sin(z);
        double ca = Math.cos(y);
        double sa = Math.sin(y);
        double cb = Math.cos(x);
        double sb = Math.sin(x);

        double m00, m01, m02, m10, m11, m12, m20, m21, m22;

        m00 = ch * sa;
        m01 = sh * sb - ch * sa * cb;
        m02 = ch * sa * sb + sh * cb;
        m10 = sa;
        m11 = ca * cb;
        m12 = -ca * sb;
        m20 = -sh * ca;
        m21 = sh * ca * cb + ch * sb;
        m22 = -sh * sa * sb + ch * cb;

        rotationMatrix.put(0, 0, m00);
        rotationMatrix.put(0, 1, m01);
        rotationMatrix.put(0 ,2, m02);
        rotationMatrix.put(1, 0, m10);
        rotationMatrix.put(1, 1, m11);
        rotationMatrix.put(1 ,2, m12);
        rotationMatrix.put(2, 0, m20);
        rotationMatrix.put(2, 1, m21);
        rotationMatrix.put(2 ,2, m22);

        return rotationMatrix;
    }

    private int fontFace = Core.FONT_ITALIC;
    private double fontScale = 0.75;
    private int fontThickness = 2;
    private int lineType = 8;
    private int radius = 4;
    private double thicknessCirc = -1;
}
