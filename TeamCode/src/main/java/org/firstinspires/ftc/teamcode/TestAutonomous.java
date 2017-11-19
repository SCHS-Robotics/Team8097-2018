package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.opencv.android.CameraBridgeViewBase;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.util.List;

/**
 * Created by test on 11/19/17.
 */

public class TestAutonomous extends Autonomous {
    private ElapsedTime runtime = new ElapsedTime();
    public void runOpMode() {

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        initialize();
        startOpenCV(this);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        runtime.reset();
        resetEncoders(motorBL, motorBR, motorFL, motorFR);

        composeTelemetry();
        // Run until the end of the match (driver presses STOP)

        try {
            goForwardDistance(100, 1);
        }
        catch (InterruptedException e) {
            System.out.println("I have no clue what just happened");
        }
    }

    public void onCameraViewStopped() {

        mRgba.release();
    }
/
    public void onCameraViewStarted(int width, int height) {

        // Creating display, color detector, defining variables for outlines (contours), not sure what a spectrum is, but we should figure that out
        mRgba = new Mat(height, width, CvType.CV_8UC4);
        mDetector = new ColorBlobDetector();
        mSpectrum = new Mat();
        CONTOUR_COLOR = new Scalar(165,255,255,255);
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
}
