package org.firstinspires.ftc.teamcode;

import android.util.Log;

import org.opencv.android.CameraBridgeViewBase;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.util.List;

import static org.firstinspires.ftc.teamcode.AutonomousNew.Position.CLOSE;
import static org.firstinspires.ftc.teamcode.AutonomousNew.Position.NOTCLOSE;
import static org.firstinspires.ftc.teamcode.AutonomousNew.Team.BLUE;
import static org.firstinspires.ftc.teamcode.AutonomousNew.Team.RED;

/**
 * Created by test on 11/19/17.
 */

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "AutonomousNew", group = "Concept")
public abstract class AutonomousNew extends BaseOpModeNew {

    public void hitJewel() {
        setArmDown();
        sleep(5000);
        switch (team) {
            case BLUE:
                break;
            case RED:
                break;
        }
    }

    public void setArmDown() {
        //TODO
    }

    public void moveToCrypto() {
        try {
            if (team == BLUE) {
                strafeLeftDistance(45, 0.5);
            } else if (team == RED) {
                strafeRightDistance(45, 0.5);
            }

            if (position == CLOSE) {
                turnTo(180, 0.5, 10);

            } else if (position == NOTCLOSE) {
                goForwardDistance(10, 0.5);
                if (team == RED) {
                    turnRightFromCurrent(45, 0.5,10);
                } else {
                    turnLeftFromCurrent(45, 0.5, 10);
                }
            }
        }
        catch (InterruptedException e) {
        }
    }

    public void onCameraViewStopped() {

        mRgba.release();
    }

    public void onCameraViewStarted(int width, int height) {

        // Creating display, color detector, defining variables for outlines (contours), not sure what a spectrum is, but we should figure that out
        mRgba = new Mat(height, width, CvType.CV_8UC4);
        mDetector = new ColorBlobDetector();
        mSpectrum = new Mat();
        CONTOUR_COLOR = new Scalar(165, 255, 255, 255);
        SPECTRUM_SIZE = new Size(200, 64);

        // The color that should be detected by default on start
        setDetectColor("brown");
    }

    public Mat onCameraFrame(CameraBridgeViewBase.CvCameraViewFrame inputFrame) {

        mRgba = inputFrame.rgba();
        if (mIsColorSelected) {
            mDetector.process(mRgba);
            List<MatOfPoint> contours = mDetector.getContours();
            Log.e("Tag", "Contours count: " + contours.size());
            Imgproc.drawContours(mRgba, contours, -1, CONTOUR_COLOR);

            Mat colorLabel = mRgba.submat(4, 68, 4, 68);
            colorLabel.setTo(mBlobColorRgba);

            Mat spectrumLabel = mRgba.submat(4, 4 + mSpectrum.rows(), 70, 70 + mSpectrum.cols());
            mSpectrum.copyTo(spectrumLabel);
        }
        return mRgba;
    }

    enum Team {
        RED,
        BLUE
    }

    enum Position {
        CLOSE,
        NOTCLOSE
    }

    public Team team;
    public Position position;
}
