package org.firstinspires.ftc.teamcode.poseDetection;

import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint3f;

import java.util.Vector;

/**
 * Created by sovnik on 11/12/17.
 */

public class Mesh {

    public Mesh() {
        id = 0;
        nVertices = 0;
        nTriangles = 0;
    }

    public Vector<Integer> getTrianglesList() {
        return listTriangles;
    }

    public MatOfPoint3f getVertex(int pos) {
        return listVertex.get(pos);
    }

    public int getNumVertices() {
        return nVertices;
    }

    public void load(String path) throws java.io.FileNotFoundException, java.io.IOException {
        CsvReader csvReader = new CsvReader(path);
        listVertex.clear();
        listTriangles.clear();

        csvReader.readPLY(listVertex, listTriangles);
        nVertices = listVertex.size();
        nTriangles = listTriangles.size();

    }

    private int id;
    private int nVertices;
    private int nTriangles;
    private Vector<MatOfPoint3f> listVertex;
    private Vector<Integer> listTriangles;
}

class Triangle {

    public Triangle(int givenId, MatOfPoint3f vertex0, MatOfPoint3f vertex1, MatOfPoint3f vertex2) {
        id = givenId;
        v0 = vertex0;
        v1 = vertex1;
        v2 = vertex2;
    }

    public MatOfPoint3f getV0() {
        return v0;
    }

    public MatOfPoint3f getV1() {
        return v1;
    }

    public MatOfPoint3f getV2() {
        return v2;
    }

    private int id;
    private MatOfPoint3f v0;
    private MatOfPoint3f v1;
    private MatOfPoint3f v2;
}

class Ray {

    public Ray(MatOfPoint3f point0, MatOfPoint3f point1) {
        p0 = point0;
        p1 = point1;
    }

    public MatOfPoint3f getP0() {
        return p0;
    }

    public MatOfPoint3f getP1() {
        return p1;
    }

    private MatOfPoint3f p0;
    private MatOfPoint3f p1;
}