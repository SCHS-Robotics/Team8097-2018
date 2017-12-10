package org.firstinspires.ftc.teamcode.poseDetection;

import com.esotericsoftware.yamlbeans.YamlException;
import com.esotericsoftware.yamlbeans.YamlReader;

import org.opencv.android.*;
import org.opencv.android.Utils;
import org.opencv.core.CvType;
import org.opencv.core.KeyPoint;
import org.opencv.core.Mat;
import org.opencv.core.MatOfKeyPoint;
import org.opencv.core.MatOfPoint3f;
import org.opencv.core.Point;
import org.opencv.core.Point3;
import org.opencv.utils.Converters;


import java.io.FileReader;
import java.util.ArrayList;
import java.util.Map;

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
        try {
            Mat points3dMat;

            YamlReader reader = new YamlReader(new FileReader(path));
            reader.getConfig().setClassTag("tag:yaml.org,2002:opencv-matrix", MatStorage.class);
            Map map = (Map) reader.read();

            MatStorage data = (MatStorage) map.get("points_3d");
            points3dMat = new Mat(data.rows, data.cols, CvType.CV_32FC1);
            points3dMat.put(0, 0, data.getData());
            points3dMat.copyTo(points3dMat);
            Converters.Mat_to_vector_Point3(points3dMat, listPoints3dIn);

            // get get get get got got got got this is a test line that I wanna try because this might be easier listPoints3dIn.add(new Point3(data.getData()));
            data = (MatStorage) map.get("descriptors");
            descriptors.put(0, 0, data.getData());
        }

        catch (java.io.FileNotFoundException | YamlException e) {
            e.printStackTrace();
        }
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

    protected static class MatStorage {
        public int rows;
        public int cols;
        public String dataType;
        public ArrayList<String> data;

        public MatStorage() {

        }

        public double[] getData() {
            double[] dataOut = new double[data.size()];

            for(int i = 0; i < dataOut.length; i++) {
                dataOut[i] = Double.parseDouble(data.get(i));
            }

            return dataOut;
        }
    }
}
