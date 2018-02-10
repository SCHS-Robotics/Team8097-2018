package org.firstinspires.ftc.teamcode;

import android.util.Log;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.opencv.android.CameraBridgeViewBase;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

/**
 * Created by test on 11/19/17.
 */
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.List;

import static org.firstinspires.ftc.teamcode.Autonomous.Position.CLOSE;
import static org.firstinspires.ftc.teamcode.Autonomous.Position.NOTCLOSE;
import static org.firstinspires.ftc.teamcode.Autonomous.Team.BLUE;
import static org.firstinspires.ftc.teamcode.Autonomous.Team.RED;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Autonomous", group = "Concept")
public abstract class Autonomous extends BaseOpMode {
    public void hitJewel() {
        setArmDown();
        sleep(5000);
        switch (team) {
            case BLUE:
                telemetry.update();
                if ((colorSensorLeft.blue() - colorSensorLeft.red() >= 7)) {
                    servoHorizontalHit.setPosition(HORIZONTAL_RIGHT_END_POS);
                    sleep(100);
                    servoVerticalHit.setPosition(VERTICAL_TELEOP_START_POS);
                } else if ((colorSensorLeft.red() - colorSensorLeft.blue() >= 7)){
                    servoHorizontalHit.setPosition(HORIZONTAL_LEFT_END_POS);
                    sleep(100);
                    servoVerticalHit.setPosition(VERTICAL_TELEOP_START_POS);
                } else {
                    servoVerticalHit.setPosition(VERTICAL_TELEOP_START_POS);
                }
            case RED:
                telemetry.update();
                if ((colorSensorLeft.red() - colorSensorLeft.blue() >= 7)) {
                    servoHorizontalHit.setPosition(HORIZONTAL_RIGHT_END_POS);
                    sleep(100);
                    servoVerticalHit.setPosition(VERTICAL_TELEOP_START_POS);
                } else if ((colorSensorLeft.blue() - colorSensorLeft.red() >= 7)){
                    servoHorizontalHit.setPosition(HORIZONTAL_LEFT_END_POS);
                    sleep(100);
                    servoVerticalHit.setPosition(VERTICAL_TELEOP_START_POS);
                } else {
                    servoVerticalHit.setPosition(VERTICAL_TELEOP_START_POS);
                }
                break;
        }
    }

    public void setArmDown() {
        servoVerticalHit.setPosition(VERTICAL_END_POS);
        servoHorizontalHit.setPosition(HORIZONTAL_END_POS);
    }

    public void moveToCrypto() {
        try {
            goForwardDistance(1, 0.25);

            if (team == BLUE) {
                turnLeftFromCurrent(90, 0.5, 5);
            } else if (team == RED) {
                turnRightFromCurrent(90, 0.5, 5);
            }

            goForwardDistance(20, .5);

            if (position == CLOSE) {
                turnTo(180, 0.5, 10);
                goForwardDistance(2, .5);
                servoTopLeftGrab.setPosition(.3);
                servoTopRightGrab.setPosition(.7);
                servoBottomLeftGrab.setPosition(.3);
                servoTopRightGrab.setPosition(.7);
                goBackwardDistance(1 , .5);

            } else if (position == NOTCLOSE) {
                goForwardDistance(10, 0.5);
                if (team == RED) {
                    turnLeftFromCurrent(90, 0.5,5);
                } else {
                    turnRightFromCurrent(90, 0.5, 5);
                }
            }
        }
        catch (InterruptedException e) {
        }
    }

    public void alignToCrypto(int num) {
        try {
            goForwardDistance(6 + (8 * (num - 1)), .5); // Hahaha this is so convoluted you know you could have just changed the return values for my distance function

            if (team == RED) {
                turnRightFromCurrent(90, 0.5, 5);
            } else {
                turnLeftFromCurrent(90, 0.5, 5);
            }

            goForwardDistance(10, .5);
            servoTopLeftGrab.setPosition(.3);
            servoTopRightGrab.setPosition(.7);
            servoBottomLeftGrab.setPosition(.3);
            servoTopRightGrab.setPosition(.7);
            goBackwardDistance(1, .5);
        } catch (InterruptedException e) {}
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

    public int targetColumnDistance(RelicRecoveryVuMark vuMarkFound) {
        switch (vuMarkFound) {
            case RIGHT:
                return 1;
            case CENTER:
                return 2;
            case LEFT:
                return 3;
        }
        return 0;
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
