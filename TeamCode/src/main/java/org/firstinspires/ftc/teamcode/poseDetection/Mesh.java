package org.firstinspires.ftc.teamcode.poseDetection;

import org.opencv.core.Mat;
import org.opencv.core.Point3;

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

    public Point3 getVertex(int pos) {
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
    private Vector<Point3> listVertex;
    private Vector<Integer> listTriangles;
}

class Triangle {

    public Triangle(int givenId, Point3 vertex0, Point3 vertex1, Point3 vertex2) {
        id = givenId;
        v0 = vertex0;
        v1 = vertex1;
        v2 = vertex2;
    }

    public Point3 getV0() {
        return v0;
    }

    public Point3 getV1() {
        return v1;
    }

    public Point3 getV2() {
        return v2;
    }

    private int id;
    private Point3 v0;
    private Point3 v1;
    private Point3 v2;
}

class Ray {

    public Ray(Point3 point0, Point3 point1) {
        p0 = point0;
        p1 = point1;
    }

    public Point3 getP0() {
        return p0;
    }

    public Point3 getP1() {
        return p1;
    }

    private Point3 p0;
    private Point3 p1;
}