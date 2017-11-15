package org.firstinspires.ftc.teamcode;

import org.opencv.core.DMatch;
import org.opencv.core.KeyPoint;
import org.opencv.core.Mat;
import org.opencv.core.MatOfDMatch;
import org.opencv.core.MatOfKeyPoint;
import org.opencv.features2d.DescriptorExtractor;
import org.opencv.features2d.DescriptorMatcher;
import org.opencv.features2d.FeatureDetector;
import org.opencv.features2d.ORB;

import java.util.ArrayList;
import java.util.Vector;

/**
 * Created by sovnik on 11/14/17.
 */

public class RobustMatcher {
    public RobustMatcher() {
        ratio = 0.8f;

        //TODO: Add in the pointer stuff once I figure out what they actually are and what they do

    }

    public void setFeatureDetector(FeatureDetector detect) {
        detector = detect;
    }

    public void setDescriptorExtractor(DescriptorExtractor desc) {
        extractor = desc;
    }

    public void setDescriptorMatcher(DescriptorMatcher match) {
        matcher = match;
    }

    public void computeKeyPoints(Mat image, MatOfKeyPoint keyPoints) {
        detector.detect(image, keyPoints);
    }

    public void computeDescriptors(Mat image, MatOfKeyPoint keyPoints, Mat descriptors) {
        extractor.compute(image, keyPoints, descriptors);
    }

    public void setRatio(float newRatio) {
        ratio = newRatio;
    }

    public int ratioTest(ArrayList<MatOfDMatch> matches) {
        int removed = 0;

        for (MatOfDMatch match : matches) {

            if (match.toList().size() > 1) {
                if (match.toList().get(0).distance / match.toList().get(1).distance > ratio) {
                    match.toList().clear();
                    removed++;
                }
            } else {
                match.toList().clear();
                removed++;
            }
        }

        return removed;
    }

    public void symmetryTest (ArrayList<MatOfDMatch> matches1, ArrayList<MatOfDMatch> matches2, MatOfDMatch symMatches) {
        for (MatOfDMatch match1 : matches1) {

            if(match1.toList().isEmpty() || match1.toList().size() < 2) {
                continue;
            }

            for (MatOfDMatch match2 : matches2) {
                if (match2.toList().isEmpty() || match2.toList().size() < 2) {
                    continue;
                }
                if (match1.toList().get(0).queryIdx == match2.toList().get(0).trainIdx && match2.toList().get(0).queryIdx == match1.toList().get(0).trainIdx) {
                    symMatches.toList().add(new DMatch(match1.toList().get(0).queryIdx, match1.toList().get(0).trainIdx, match1.toList().get(0).distance));
                    break;
                }
            }
        }
    }

    public void robustMatch (Mat frame, MatOfDMatch goodMatches, MatOfKeyPoint keyPointsFrame, Mat descriptorsModel) {
        this.computeKeyPoints(frame, keyPointsFrame);

        Mat descriptorsFrame = new Mat();
        this.computeDescriptors(frame, keyPointsFrame, descriptorsFrame);

        ArrayList<MatOfDMatch> matches12 = new ArrayList<>();
        ArrayList<MatOfDMatch> matches21 = new ArrayList<>();
        
        matcher.knnMatch(descriptorsFrame, descriptorsModel, matches12, 2);

        matcher.knnMatch(descriptorsModel, descriptorsFrame, matches21, 2);

        ratioTest(matches12);
        ratioTest(matches21);

        symmetryTest(matches12, matches21, goodMatches);

    }

    public void fastRobustMatch (Mat frame, MatOfDMatch goodMatches, MatOfKeyPoint keyPointsFrame, Mat descriptorsModel) {
        goodMatches.toList().clear();
        this.computeKeyPoints(frame, keyPointsFrame);

        Mat descriptorsFrame = new Mat();
        this.computeDescriptors(frame, keyPointsFrame, descriptorsFrame);

        ArrayList<MatOfDMatch> matches = new ArrayList<>();
        matcher.knnMatch(descriptorsFrame, descriptorsModel, matches, 2);

        ratioTest(matches);

        for (MatOfDMatch match : matches) {
            if (match.toList().isEmpty()) {
                goodMatches.toList().add(match.toList().get(0));
            }
        }
    }

    private FeatureDetector detector;

    private DescriptorExtractor extractor;

    private DescriptorMatcher matcher;

    private float ratio;
}
