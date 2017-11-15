package org.firstinspires.ftc.teamcode;

import org.opencv.core.Point3;

import java.io.BufferedReader;
import java.io.FileReader;
import java.util.ArrayList;
import java.util.Scanner;

/**
 * Created by sovnik on 11/14/17.
 */

public class CsvReader {

    public CsvReader(String path) throws java.io.FileNotFoundException{
        file = new FileReader(path);
        scanner = new Scanner(file);
    }

    public void readPLY(ArrayList<Point3> listVertex, ArrayList<ArrayList<Integer>> listTriangles) {
        String line;
        String tmpStr;
        String n;

        int nVertex = 0;
        int nTriangles = 0;
        int count = 0;

        boolean endHeader = false;
        boolean endVertex = false;

        while (scanner.hasNext()) {
            scanner.useDelimiter(separator);

            if(!endHeader) {
                tmpStr = scanner.nextLine();

                if (tmpStr == "element") {
                    tmpStr= scanner.nextLine();
                    n = scanner.nextLine();
                    if (tmpStr == "vertex") {
                        nVertex = Integer.parseInt(n);
                    }
                    if (tmpStr == "face") {
                        nTriangles = Integer.parseInt(n);
                    }
                }

                if (tmpStr == "end_header") {
                    endHeader = true;
                }

            }

            else if(endHeader) {
                String x;
                String y;
                String z;

                x = scanner.nextLine();
                y = scanner.nextLine();
                z = scanner.nextLine();

                Point3 tmpPoint = new Point3();
                tmpPoint.x = Integer.parseInt(x);
                tmpPoint.y = Integer.parseInt(y);
                tmpPoint.z = Integer.parseInt(z);
                listVertex.add(tmpPoint);

                count++;
                if (count == nVertex) {
                    count = 0;
                    endVertex = true;
                }
            }

            else if(endVertex && count < nTriangles) {
                String nPointsPerFace;
                String id0;
                String id1;
                String id2;

                nPointsPerFace = scanner.nextLine();
                id0 = scanner.nextLine();
                id1 = scanner.nextLine();
                id2 = scanner.nextLine();

                ArrayList<Integer> tmpTriangle = new ArrayList<>(3);
                tmpTriangle.set(0, Integer.parseInt(id0));
                tmpTriangle.set(1, Integer.parseInt(id1));
                tmpTriangle.set(2, Integer.parseInt(id2));
                listTriangles.add(tmpTriangle);

                count++;
            }
        }
    }

    private String separator = " ";
    private FileReader file;
    private Scanner scanner;
    private BufferedReader reader;
}
