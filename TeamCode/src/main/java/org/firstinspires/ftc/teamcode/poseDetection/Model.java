package org.firstinspires.ftc.teamcode.poseDetection;

import org.opencv.core.KeyPoint;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint2f;

import java.io.FileReader;
import java.util.Vector;

import org.opencv.core.MatOfPoint3f;
import org.opencv.features2d.*;
import org.opencv.features2d.Features2d;

/**
 * Created by sovnik on 11/11/17.
 */

public class Model {

    public Vector<MatOfPoint2f> getPoints2dIn() {
        return listPoints2dIn;
    }

    public Vector<MatOfPoint2f> getPoints2dOut() {
        return listPoints2dOut;
    }

    public Vector<MatOfPoint3f> getPoints3d() {
        return listPoints3dIn;
    }

    public Vector<KeyPoint> getKeyPoints() {
        return listKeypoints;
    }

    public Mat getDescriptors() {
        return descriptors;
    }

    int getNumDescriptors() {
        return descriptors.rows();
    }

    public void addCorrespondence(MatOfPoint2f point2d, MatOfPoint3f point3d){
        listPoints2dIn.addElement(point2d);
        listPoints3dIn.addElement(point3d);
        nCorrespondences++;
    }

    public void addOutlier(MatOfPoint2f point2d){
        listPoints2dOut.addElement(point2d);
    }

    public void addDescriptor(Mat descriptor){
        descriptors.push_back(descriptor);
    }

    public void addKeypoint(KeyPoint kp){
        listKeypoints.addElement(kp);
    }

    public void save(String path){

    }

    public void load(String path){

    }

    private int nCorrespondences = 0;
    private Vector<KeyPoint> listKeypoints;
    private Vector<MatOfPoint2f> listPoints2dIn;
    private Vector<MatOfPoint2f> listPoints2dOut;
    private Vector<MatOfPoint3f> listPoints3dIn;
    private Mat descriptors;
}
