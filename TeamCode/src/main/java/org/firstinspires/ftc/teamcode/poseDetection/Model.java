package org.firstinspires.ftc.teamcode.poseDetection;

import org.opencv.core.KeyPoint;
import org.opencv.core.Mat;
import org.opencv.core.MatOfKeyPoint;
import org.opencv.core.Point;
import org.opencv.core.Point3;

import java.util.ArrayList;

/**
 * Created by sovnik on 11/14/17.
 */

public class Model {

    public Model() {
        nCorrespondences = 0;

    }

    public void addCorrespondence(Point point2d, Point3 point3d) {
        listPoints2dIn.add(point2d);
        listPoints3dIn.add(point3d);
        nCorrespondences++;
    }

    public void addOutlier(Point point2d) {
        listPoints2dOut.add(point2d);
    }

    public void addDescriptor(Mat descriptor) {
        descriptors.push_back(descriptor);
    }

    public void addKeypoint(KeyPoint kp) {
        listKeypoints.toList().add(kp);
    }

    public void load(String path) {
        Mat points3dMat;

        // TODO: Make a YML parser sometime or this won't work
        // Alternatively, generate the YML files and just manually put the values in here.
    }

    public ArrayList<Point> getPoints2dIn() {
        return listPoints2dIn;
    }

    public ArrayList<Point> getPoints2dOut() {
        return listPoints2dOut;
    }

    public ArrayList<Point3> getPoints3d() {
        return listPoints3dIn;
    }

    public MatOfKeyPoint getKeypoints() {
        return listKeypoints;
    }

    public Mat getDescriptors() {
        return descriptors;
    }

    public int getNumDescriptors() {
        return descriptors.rows();
    }

    private int nCorrespondences;
    private MatOfKeyPoint listKeypoints;
    private ArrayList<Point> listPoints2dIn;
    private ArrayList<Point> listPoints2dOut;
    private ArrayList<Point3> listPoints3dIn;
    private Mat descriptors;
}
