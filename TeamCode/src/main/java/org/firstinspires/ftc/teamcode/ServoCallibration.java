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

import com.qualcomm.robotcore.eventloop.opmode.*;
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
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */
@TeleOp(name="ServoCallibration", group="Linear Opmode")
public class ServoCallibration extends BaseOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    // Button refreshes, at some point there should be a better way to do this, but at the moment there is not so don't complain.
    private int                 buttonACooldown;
    private int                 buttonBCooldown;
    private int                 buttonXCooldown;


    private int                 selectedAngle = 0;

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
        while (opModeIsActive()) {
            // Telemetry fun
            telemetry.update();
            telemetry.addData("Servo Left Grab Pos: ", servoLeftGrab.getPosition());
            telemetry.addData("Servo Right Grab Pos: ", servoRightGrab.getPosition());
            telemetry.addData("Selected turn angle: ", selectedAngle);
            telemetry.addData("Left Stick X: ", gamepad1.left_stick_x);
            telemetry.addData("Left Stick Y: ", gamepad1.left_stick_y);
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Glyph Detection Color", detectColor);
//            telemetry.addData("Left Lift Position", motorLeftLift.getCurrentPosition());
//            telemetry.addData("Right Lift Position", motorRightLift.getCurrentPosition());


            if(gamepad1.a && buttonACooldown >= 1000) {
                if (servoLeftGrab.getPosition() > .5 && servoRightGrab.getPosition() < .5) {
                    servoLeftGrab.setPosition(0.3);
                    servoRightGrab.setPosition(0.7);
                } else {
                    servoLeftGrab.setPosition(1);
                    servoRightGrab.setPosition(0);
                }
                buttonACooldown = 0;
            }

//            if (gamepad1.dpad_left) {
//                servoLeftGrab.setPosition(servoLeftGrab.getPosition() - .001);
//            }
//
//            if (gamepad1.dpad_right) {
//                servoLeftGrab.setPosition(servoLeftGrab.getPosition() + .001);
//            }
//
//            if (gamepad1.dpad_up) {
//                servoRightGrab.setPosition(servoRightGrab.getPosition() - .001);
//            }
//
//            if (gamepad1.dpad_down) {
//                servoRightGrab.setPosition(servoRightGrab.getPosition() + .001);
//            }



            if(buttonACooldown < 1000){
                buttonACooldown++;
            }

            if(gamepad1.b && buttonBCooldown >= 500) {
                switch (detectColor){
                    case "brown": setDetectColor("gray");
                        break;
                    case "gray": setDetectColor("red");
                        break;
                    case "red": setDetectColor("blue");
                        break;
                    case "blue": setDetectColor("brown");
                        break;
                }
                buttonBCooldown = 0;
            }

            if(buttonBCooldown < 500){
                buttonBCooldown++;
            }

//            if (gamepad1.right_bumper) {
//                motorLeftLift.setPower(.001);
//                motorRightLift.setPower(-.001);
//            }
//
//            if (gamepad1.left_bumper) {
//                motorRightLift.setPower(.001);
//                motorLeftLift.setPower(-.001);
//            }

            if(buttonXCooldown < 500){
                buttonXCooldown++;
            }
        }

        stopOpenCV();

    }

    // Direction Functions
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

    public void selectTurnAngle() {

        switch (selectedAngle) {
            case 0: selectedAngle = 90;
                break;
            case 90: selectedAngle = 180;
                break;
            case 180: selectedAngle = 270;
                break;
            case 270: selectedAngle = 0;
                break;
        }
    }

    // OpenCV functions
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
}