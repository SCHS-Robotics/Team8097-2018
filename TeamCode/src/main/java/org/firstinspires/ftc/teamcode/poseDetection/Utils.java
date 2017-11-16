package org.firstinspires.ftc.teamcode.poseDetection;

import org.opencv.calib3d.Calib3d;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Point3;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;

import static org.opencv.core.CvType.CV_64F;

/**
 * Created by sovnik on 11/15/17.
 */

public class Utils {

    public static void drawText(Mat image, String text, Scalar color) {
        Imgproc.putText(image, text, new Point(25, 50), fontFace, fontScale, color, fontThickness);
    }

    public static void drawConfidence(Mat image, double confidence, Scalar color) {
        String confStr = Integer.toString((int) confidence);
        String text = confStr + "%";
        Imgproc.putText(image, text, new Point(500, 50), fontFace, fontScale, color, fontThickness);
    }

    public static void draw2dPoints(Mat image, List<Point> listPoints, Scalar color) {
        for(int i = 0; i < listPoints.size(); i++) {
            Point point2d = listPoints.get(i);
            Imgproc.circle(image, point2d, radius, color, -1);
        }
    }

    public static void draw3dCoordinateAxes(Mat image, ArrayList<Point> listPoints2d) {
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

    public static void drawObjectMesh(Mat image, Mesh mesh, PnPProblem pnPProblem, Scalar color) {
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

    public static Mat eulerToRot(Mat euler) {
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

    private static int fontFace = Core.FONT_ITALIC;
    private static double fontScale = 0.75;
    private static int fontThickness = 2;
    private static int lineType = 8;
    private static int radius = 4;
    private double thicknessCirc = -1;
}
