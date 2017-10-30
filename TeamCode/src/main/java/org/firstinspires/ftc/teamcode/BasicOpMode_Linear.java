/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import android.util.Log;
import android.view.MotionEvent;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;
import org.opencv.android.CameraBridgeViewBase;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.opencv.videoio.VideoCapture;

import java.util.ConcurrentModificationException;
import java.util.List;


/**
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */
@TeleOp(name="Basic: Linear OpMode", group="Linear Opmode")
public class BasicOpMode_Linear extends BaseOpModeTest {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    private boolean              mIsColorSelected = true;
    private Mat                  mRgba;
    private Scalar               mBlobColorRgba;
    private Scalar               mBlobColorHsv;
    private ColorBlobDetector    mDetector;
    private Mat                  mSpectrum;
    private Size                 SPECTRUM_SIZE;
    private Scalar               CONTOUR_COLOR;
    private Rect                 returnedBoundingRect;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        initialize();

        startOpenCV(this);


        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
//            for (MatOfPoint contour: mDetector.getContours()) {
//                returnedBoundingRect = Imgproc.boundingRect(contour);
//            }

            double inputX = gamepad1.left_stick_x;
            double inputY = gamepad1.left_stick_y;
            double inputMag = Math.abs(Math.sqrt(Math.pow(inputX, 2) + Math.pow(inputY, 2)));
            double angle = Math.toDegrees(Math.atan2(inputY, inputX));
            if (Math.abs(inputX) >= .1 || Math.abs(inputY) >= .1){
                try {
                    goDirection(inputMag, angle);
                }
                catch (java.lang.InterruptedException a) {
                    stop();
                }
            }
            else if (gamepad1.left_trigger > .1){
                turnLeft(gamepad1.left_trigger);
            }
            else if (gamepad1.right_trigger > .1){
                turnRight(gamepad1.right_trigger);
            }
            else {
                motorBL.setPower(0);
                motorBR.setPower(0);
                motorFL.setPower(0);
                motorFR.setPower(0);
            }

            if (gamepad1.dpad_up) {
                //servoCamera.setPosition(servoCamera.getPosition() + .001);
                servoLeftGrab.setPosition(servoLeftGrab.getPosition() + .01);
            }
            else if (gamepad1.dpad_down){
                //servoCamera.setPosition(servoCamera.getPosition() - .001);
                servoLeftGrab.setPosition(servoLeftGrab.getPosition()-.01);
            }

            if (gamepad1.dpad_left) {
                //servoCamera.setPosition(servoCamera.getPosition() + .001);
                servoRightGrab.setPosition(servoRightGrab.getPosition()+.01);
            }
            else if (gamepad1.dpad_right){
                //servoCamera.setPosition(servoCamera.getPosition() - .001);
                servoRightGrab.setPosition(servoRightGrab.getPosition()-.01);
            }



            telemetry.addData("Servo Camera Pos: ", servoCamera.getPosition());
            telemetry.addData("Servo Left Grab Pos: ", servoLeftGrab.getPosition());
            telemetry.addData("Servo Right Grab Pos: ", servoRightGrab.getPosition());
            telemetry.addData("Angle of Left Joystick: ", angle);
            telemetry.addData("Left Stick X: ", gamepad1.left_stick_x);
            telemetry.addData("Left Stick Y: ", gamepad1.left_stick_y);
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            //telemetry.addData("Status", "Bounding Rect: " + returnedBoundingRect.toString());
            telemetry.update();
        }

    }
    public boolean angleIsNearAngle(double angle1, double angle2) {
        while (angle1 >= 360) {
            angle1 -= 360;
        }
        while (angle1 < 0) {
            angle1 += 360;
        }
        while (angle2 >= 360) {
            angle2 -= 360;
        }
        while (angle2 < 0) {
            angle2 += 360;
        }
        double diff = Math.abs(angle2 - angle1);
        return diff <= 45.0 / 2 || diff >= 360 - 45.0 / 2;
    }

    public void goDirection(double magnitude, double angle) throws InterruptedException {
        if (angleIsNearAngle(angle, 0)) {
            goRight(magnitude);
        } else if (angleIsNearAngle(angle, 45)) {
            goDiagonalForwardRight(magnitude);
        } else if (angleIsNearAngle(angle, 90)) {
            goForward(magnitude);
        } else if (angleIsNearAngle(angle, 135)) {
            goDiagonalForwardLeft(magnitude);
        } else if (angleIsNearAngle(angle, 180)) {
            goLeft(magnitude);
        } else if (angleIsNearAngle(angle, 225)) {
            goDiagonalBackwardLeft(magnitude);
        } else if (angleIsNearAngle(angle, 270)) {
            goBackward(magnitude);
        } else if (angleIsNearAngle(angle, 315)) {
            goDiagonalBackwardRight(magnitude);
        } else {
            System.out.print("Where will this go");
        }
    }

    public void onCameraViewStarted(int width, int height) {
        mRgba = new Mat(height, width, CvType.CV_8UC4);
        mDetector = new ColorBlobDetector();
        mSpectrum = new Mat();
        CONTOUR_COLOR = new Scalar(255,0,0,255);
        SPECTRUM_SIZE = new Size(200, 64);

        // Weeeeee assign values to the color scalars weeeee (I'm not actually sure why we have to do this but every example I've seen did the same thing)
        mBlobColorRgba = new Scalar(255);
        mBlobColorHsv = new Scalar(255);

        // and then change them immediately :DDDDDDDDD
        mBlobColorHsv = new Scalar(195, 255, 255);
        mBlobColorRgba = convertScalarHsv2Rgba(mBlobColorHsv);

        mDetector.setHsvColor(mBlobColorHsv);

        mIsColorSelected = true;


    }

    public void onCameraViewStopped() {
        mRgba.release();
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

    private Scalar convertScalarHsv2Rgba(Scalar hsvColor) {
        Mat pointMatRgba = new Mat();
        Mat pointMatHsv = new Mat(1, 1, CvType.CV_8UC3, hsvColor);
        Imgproc.cvtColor(pointMatHsv, pointMatRgba, Imgproc.COLOR_HSV2RGB_FULL, 4);

        return new Scalar(pointMatRgba.get(0, 0));
    }

}