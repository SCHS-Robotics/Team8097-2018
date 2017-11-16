package org.firstinspires.ftc.teamcode.poseDetection;

import org.opencv.core.Point3;

import java.util.ArrayList;

/**
 * Created by sovnik on 11/14/17.
 */

public class Mesh {

    public Mesh() {
        id = 0;
        nVertex = 0;
        nTriangles = 0;
    }

    public ArrayList<ArrayList<Integer>> getTrianglesList() {
        return listTriangles;
    }

    public Point3 getvertex(int pos) {
        return listVertex.get(pos);
    }

    public int getNumVertices() {
        return nVertex;
    }

    public void load(String path) throws java.io.FileNotFoundException {
        CsvReader csvReader = new CsvReader(path);

        listVertex.clear();;
        listTriangles.clear();

        csvReader.readPLY(listVertex, listTriangles);

        nVertex = listVertex.size();
        nTriangles = listTriangles.size();

    }

    private int id;
    private int nVertex;
    private int nTriangles;
    private ArrayList<Point3> listVertex;
    private ArrayList<ArrayList<Integer>> listTriangles;

}

class Triangle {

    public Triangle(int newId, Point3 vertex0, Point3 vertex1, Point3 vertex2) {
        id = newId;
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
