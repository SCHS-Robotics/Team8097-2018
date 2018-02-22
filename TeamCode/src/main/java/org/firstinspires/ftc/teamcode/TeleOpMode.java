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

import com.qualcomm.robotcore.util.ElapsedTime;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="TeleOpMode", group="Linear Opmode")
public class TeleOpMode extends BaseOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime cooldown = new ElapsedTime();

    // Button refreshes, at some point there should be a better way to do this, but at the moment there is not so don't complain.
    private double                 buttonACooldown;
    private double                 buttonXCooldown;
    private double                 buttonYCooldown;
    private double                 buttonLBCooldown;
    private double                 buttonRBCooldown;

    @Override
    public void runOpMode() {

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        liftState = LiftState.DOWN;
        grabStatus = GrabStatus.OPEN;
        hitStatus = HitStatus.DOWN;

        initialize();
        servoTopLeftGrab.setPosition(1);
        servoTopRightGrab.setPosition(0);
        servoBottomLeftGrab.setPosition(1);
        servoTopRightGrab.setPosition(0);

        servoHorizontalHit.setPosition(HORIZONTAL_AUTO_START_POS);
        servoVerticalHit.setPosition(VERTICAL_AUTO_START_POS);
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        runtime.reset();
        cooldown.reset();
        resetEncoders(motorBL, motorBR, motorFL, motorFR, motorLeftLift, motorRightLift);

        // Run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double inputX = gamepad1.left_stick_x;
            double inputY = gamepad1.left_stick_y;
            double inputMag = Math.abs(Math.sqrt(Math.pow(inputX, 2) + Math.pow(inputY, 2)));
            double angle = Math.toDegrees(Math.atan2(inputY, inputX));

            // Telemetry fun
            telemetry.update();

            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Servo Vertical", servoVerticalHit.getPosition());
            telemetry.addData("Left Lift Pos", motorLeftLift.getCurrentPosition());
            telemetry.addData("Right Lift Pos", motorRightLift.getCurrentPosition());
            telemetry.addData("Motor BL Pos", motorBL.getCurrentPosition());
            telemetry.addData("Motor BR Pos", motorBR.getCurrentPosition());
            telemetry.addData("Motor FL Pos", motorFL.getCurrentPosition());
            telemetry.addData("Motor FR Pos", motorFR.getCurrentPosition());

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

            if (gamepad1.dpad_down) {
                motorLeftLift.setPower(-.2);
                motorRightLift.setPower(.2);
            }
            else if (gamepad1.dpad_up){
                 motorLeftLift.setPower(.2);
                 motorRightLift.setPower(-.2);
             }
             else {
                motorRightLift.setPower(0);
                motorLeftLift.setPower(0);
            }

            if(gamepad1.a && Math.abs(cooldown.time() - buttonACooldown) >= .2) {
                if (grabStatus == GrabStatus.OPEN) {
                    grabStatus = GrabStatus.CLOSE;
                    servoTopLeftGrab.setPosition(0.3);
                    servoTopRightGrab.setPosition(0.7);
                    servoBottomLeftGrab.setPosition(0.3);
                    servoBottomRightGrab.setPosition(0.7);
                } else if (grabStatus == GrabStatus.HALFOPEN){
                    grabStatus = GrabStatus.OPEN;
                    servoTopLeftGrab.setPosition(1);
                    servoTopRightGrab.setPosition(0);
                    servoBottomLeftGrab.setPosition(1);
                    servoBottomRightGrab.setPosition(0);
                } else {
                    grabStatus = GrabStatus.HALFOPEN;
                    servoTopLeftGrab.setPosition(.6);
                    servoTopRightGrab.setPosition(.4);
                    servoBottomLeftGrab.setPosition(.6);
                    servoBottomRightGrab.setPosition(.4);
                }
                buttonACooldown = cooldown.time();
            }

            if (gamepad1.x && Math.abs(cooldown.time() - buttonXCooldown) >= 1) {
                try {
                    toggleLift();
                } catch (InterruptedException e) {}

                buttonXCooldown = cooldown.time();
            }

            if (gamepad1.y && Math.abs(cooldown.time() - buttonYCooldown) >= 1) {
                if (hitStatus == HitStatus.DOWN) {
                    hitStatus = HitStatus.UP;
                    servoHorizontalHit.setPosition(HORIZONTAL_TELEOP_START_POS);
                    servoVerticalHit.setPosition(VERTICAL_TELEOP_START_POS);
                }
                else if (hitStatus == HitStatus.UP) {
                    hitStatus = HitStatus.DOWN;
                    servoHorizontalHit.setPosition(HORIZONTAL_END_POS);
                    while (Math.abs(servoVerticalHit.getPosition() - VERTICAL_END_POS) >= .01) {
                        servoVerticalHit.setPosition(servoVerticalHit.getPosition() + .005);
                    }
                }
                buttonYCooldown = cooldown.time();
            }

            if (gamepad1.dpad_left && hitStatus == HitStatus.DOWN) {
                servoHorizontalHit.setPosition(HORIZONTAL_LEFT_END_POS);
            }
            else if (gamepad1.dpad_right && hitStatus == HitStatus.DOWN) {
                servoHorizontalHit.setPosition(HORIZONTAL_RIGHT_END_POS);
            }

            if (gamepad1.left_bumper && Math.abs(cooldown.time() - buttonLBCooldown) >= 1) {
                turnLeftFromCurrent(90, 0.75, 15);
                buttonLBCooldown = cooldown.time();
            }

            if (gamepad1.right_bumper && Math.abs(cooldown.time() - buttonRBCooldown) >= 1) {
                turnRightFromCurrent(90, 0.75, 15);
                buttonRBCooldown = cooldown.time();
            }
        }
    }

    // Direction Functions
    private boolean angleIsNearAngle(double angle1, double angle2) {

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

    private void goDirection(double magnitude, double angle) throws InterruptedException {

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
}