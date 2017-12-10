package org.firstinspires.ftc.teamcode.poseDetection;

import org.opencv.core.Point3;

import java.io.FileReader;
import java.util.ArrayList;
import java.util.Scanner;

/**
 * Created by sovnik on 11/20/17.
 */

public class CsvReader {
    public CsvReader(String path) throws java.io.FileNotFoundException{
        file = new FileReader(path);
        reader = new Scanner(file);
    }

    public void readPLY(ArrayList<Point3> listVertex, ArrayList<ArrayList<Integer>> listTriangles) {
        String tmpStr;
        int nVertex = 0;
        int nTriangles = 0;
        int count = 0;
        boolean endHeader = false;
        boolean endVertex = false;


        while(reader.hasNext()) {
            if (!endHeader) {
                tmpStr = reader.nextLine();
                if (tmpStr.contains("element")) {
                    String[] splitStr = tmpStr.split(" ");
                    if (splitStr[1].equals("face")) {
                        nTriangles = Integer.parseInt(splitStr[2]);
                    }
                    if (splitStr[1].equals("vertex")) {
                        nVertex = Integer.parseInt(splitStr[2]);
                    }
                }

                if (tmpStr.contains("end_header")) {
                    endHeader = true;
                }
            }

            else if (endHeader) {
                if (!endVertex && count < nVertex) {
                    tmpStr = reader.nextLine();
                    String[] splitStr = tmpStr.split(" ");
                    Point3 tmpPoint = new Point3();
                    tmpPoint.x = Float.parseFloat(splitStr[0]);
                    tmpPoint.y = Float.parseFloat(splitStr[1]);
                    tmpPoint.z = Float.parseFloat(splitStr[2]);
                    listVertex.add(tmpPoint);

                    count++;

                    if (count == nVertex) {
                        count = 0;
                        endVertex = true;
                    }
                }

                else if (endVertex && count < nTriangles) {
                    int id0, id1, id2;
                    tmpStr = reader.nextLine();
                    String[] splitStr = tmpStr.split(" ");
                    id0 = Integer.parseInt(splitStr[1]);
                    id1 = Integer.parseInt(splitStr[2]);
                    id2 = Integer.parseInt(splitStr[3]);
                    ArrayList<Integer> tmpTriangle = new ArrayList<>(3);

                    tmpTriangle.add(0, id0);
                    tmpTriangle.add(1, id1);
                    tmpTriangle.add(2, id2);
                    listTriangles.add(tmpTriangle);

                    count++;
                }
            }
        }
    }

    private FileReader file;
    Scanner reader;
}

