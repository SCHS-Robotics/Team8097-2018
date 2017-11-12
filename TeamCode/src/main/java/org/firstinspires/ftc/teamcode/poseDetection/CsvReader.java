package org.firstinspires.ftc.teamcode.poseDetection;

import org.opencv.core.MatOfPoint3;
import org.opencv.core.MatOfPoint3f;
import org.opencv.core.Point3;

import java.io.BufferedInputStream;
import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;
import java.io.InputStream;
import java.util.Scanner;
import java.util.Vector;

/**
 * Created by sovnik on 11/12/17.
 */

public class CsvReader {

    public CsvReader(String path) throws java.io.FileNotFoundException{
        file = new FileReader(path);
        reader = new BufferedReader(file);
        scanner = new Scanner(reader);

    }

    public void readPLY(Vector<MatOfPoint3f> listVertex, Vector<Integer> listTriangles) throws java.io.IOException {
        // Read variables
        String line = "";
        String tmpStr = "";
        String n = "";
        // Search variables
        int nVertex = 0;
        int nTriangles = 0;
        int count = 0;
        // Condition checks for sections of the .ply file
        boolean endHeader = false;
        boolean endVertex = false;


        while (scanner.hasNext()){
            Scanner liness = new Scanner(scanner.nextLine());
            liness.useDelimiter(separator);

            // Parses the header
            if (!endHeader) {
                tmpStr = liness.nextLine();

                // Looks for relevant elements to add (Come in sets of 3, first line declaring an element, second declaring an element type, third giving a number)
                if (tmpStr == "element"){
                    // Reads through the lines following "element" two at a time
                    tmpStr = liness.nextLine();
                    n = liness.nextLine();
                    // If the first of the two read lines detects a vertex, set the second to the total number of vertices.
                    if(tmpStr == "vertex"){
                        nVertex = Integer.parseInt(n);
                    }
                    // If the first detects a face, set the second to total triangles.
                    if(tmpStr == "face"){
                        nTriangles = Integer.parseInt(n);
                    }
                }

                // Checks to see if the header is finished
                if (tmpStr == "end_header") {
                    endHeader = true;
                }
            }

            // Starts reading the rest
            else if(endHeader) {

                //Adds each of the vertices to the vector
                if(!endVertex && count < nVertex) {
                    String x;
                    String y;
                    String z;
                    x = liness.nextLine();
                    y = liness.nextLine();
                    z = liness.nextLine();

                    Point3 tmpPoint = new Point3();
                    tmpPoint.x = (float)Integer.parseInt(x);
                    tmpPoint.y = (float)Integer.parseInt(y);
                    tmpPoint.z = (float)Integer.parseInt(z);
                    MatOfPoint3f tmpPointMat = new MatOfPoint3f(tmpPoint);
                    listVertex.add(tmpPointMat);

                    count++;

                    if (count == nVertex) {
                        count = 0;
                        endVertex = true;
                    }
                }

                // Adds each of the triangles to the vector
                else if(endVertex && count < nTriangles) {
                    String nPointsPerFace;
                    String id0;
                    String id1;
                    String id2;

                    nPointsPerFace = liness.nextLine();
                    id0 = liness.nextLine();
                    id1 = liness.nextLine();
                    id2 = liness.nextLine();

                    Vector<Integer> tmpTriangle = new Vector<>(3);
                    tmpTriangle.setElementAt(Integer.parseInt(id0), 0);
                    tmpTriangle.setElementAt(Integer.parseInt(id1), 1);
                    tmpTriangle.setElementAt(Integer.parseInt(id2), 2);
                    listTriangles.addAll(tmpTriangle);

                    count++;
                }
            }
        }
    }

    private final String separator = " ";
    private FileReader file;
    private Scanner scanner;
    private BufferedReader reader;
}
