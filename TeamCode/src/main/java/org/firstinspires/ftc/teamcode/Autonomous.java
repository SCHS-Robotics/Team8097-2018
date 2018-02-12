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

import android.speech.tts.TextToSpeech;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Locale;

import static org.firstinspires.ftc.teamcode.Autonomous.Position.CLOSE;
import static org.firstinspires.ftc.teamcode.Autonomous.Position.NOTCLOSE;
import static org.firstinspires.ftc.teamcode.Autonomous.Team.BLUE;
import static org.firstinspires.ftc.teamcode.Autonomous.Team.RED;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Autonomous", group = "Concept")
public abstract class Autonomous extends BaseOpMode {

    private VuforiaLocalizer vuforia = null;
    private VuforiaTrackables relicTrackables = null;
    private VuforiaTrackable relicTemplate = null;
    private RelicRecoveryVuMark foundVuMark = RelicRecoveryVuMark.UNKNOWN;
    ElapsedTime runtime = new ElapsedTime();

    void hitJewel() {
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

    void setArmDown() {
        servoVerticalHit.setPosition(VERTICAL_END_POS);
        servoHorizontalHit.setPosition(HORIZONTAL_END_POS);
    }

    void initGrabServos() {
        servoTopLeftGrab.setPosition(1);
        servoTopRightGrab.setPosition(0);
        servoBottomLeftGrab.setPosition(1);
        servoBottomRightGrab.setPosition(0);
    }

    void setArmUp() {
        servoVerticalHit.setPosition(VERTICAL_AUTO_START_POS);
        servoHorizontalHit.setPosition(HORIZONTAL_AUTO_START_POS);
    }

    void raiseLift() {
        motorLeftLift.setPower(0.15);
        motorRightLift.setPower(-0.15);
        sleep(1000);
        motorRightLift.setPower(0);
        motorLeftLift.setPower(0);
    }

    void startPositioning() {
        boolean complete = false;
        while(complete == false) {
            double distance = rangeSensor.getDistance(DistanceUnit.CM);
            if (distance > JEWEL_ARM_MAX_DISTANCE) {
                telemetry.addLine("Robot too far!");
            }
            else if (distance < JEWEL_ARM_MIN_DISTANCE) {
                telemetry.addLine("Robot too close!");
            } else {
                telemetry.addLine("Robot at optimal distance");
            }
            telemetry.addData("Current distance: ", distance);
            telemetry.addLine("Press A to accept positioning");
            telemetry.update();

            if(gamepad1.a) {
                complete = true;
            }
        }
    }

    void moveToCrypto() {
        try {
            goForwardDistance(1, 0.25);

            if (team == BLUE) {
                turnLeftFromCurrent(90, 0.5, 5);
            } else if (team == RED) {
                turnRightFromCurrent(90, 0.5, 5);
            }

            goForwardDistance(20, .5);

            if (position == CLOSE) {

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

    void alignToCrypto(int targetColumn) {
        try {
            if (position == CLOSE) {
                goForwardDistance(targetColumn, .5);
                turnTo(180, 0.5, 10);
            }
            if (position == NOTCLOSE) {
                goForwardDistance(6 + (8 + targetColumn), .5); // Hahaha this is so convoluted you know you could have just changed the return values for my distance function

                if (team == RED) {
                    turnRightFromCurrent(90, 0.5, 5);
                } else {
                    turnLeftFromCurrent(90, 0.5, 5);
                }
            }
            goForwardDistance(10, .5);
            servoTopLeftGrab.setPosition(.3);
            servoTopRightGrab.setPosition(.7);
            servoBottomLeftGrab.setPosition(.3);
            servoBottomRightGrab.setPosition(.7);
            goBackwardDistance(3, .5);
        }
        catch(InterruptedException e){}
    }

    void initializeVuforia() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.vuforiaLicenseKey = "AWRsObH/////AAAAGU5bp4bnDkCYjwnKsD5okRCL7t6ejVuLHi3TwTkPTSo+EuLnlmB+G2Rz4GOel217l0cjjlYjJfot5pvsspqgEUJvtNDeoOacTA3bzKaeAFUoBeQA2r3VwolpdWR/6xxq9EraYiLIkOLee51c2Uqtzlvk8Qav301W2TJOdPbotZUAndR6QlIQ7m2UVZWY+2qlenB36jIF3ZGotK/QwihY0/96KWzHtbIPUheU4CiJmRlIi3xMGREt3SYgcPV3L/WMPi+WW7GSSoh9IVaVnfGmTZD2cWSGeB/x4RDHdUbePjZrEQ1OPNR/LvjbRYWkX+QgQUqmyff0/Etuf9o0oJ9PlYeeteZPv1m/2hiB/mUG9tz1";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        vuforia = ClassFactory.createVuforiaLocalizer(parameters);
        relicTrackables = vuforia.loadTrackablesFromAsset("RelicVuMark");
        relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate");
    }

    void startVuforia() {
        relicTrackables.activate();
    }

    private RelicRecoveryVuMark getVuMark() {
        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
        return vuMark;
    }

    RelicRecoveryVuMark getFoundVuMark() {
        return foundVuMark;
    }

    void vuforiaDetect() {
        double vuTimer = runtime.time();
        do {
            foundVuMark = getVuMark();
            if (runtime.time() - vuTimer >= 5) {
                telemetry.addLine("VuMark Not Found in time, giving up and blaming hardware");
                telemetry.update();
                break;
            }
        } while(foundVuMark == RelicRecoveryVuMark.UNKNOWN);
        telemetry.addData("VuMark: ", foundVuMark);
    }

    int targetColumnDistance(RelicRecoveryVuMark vuMarkFound) {
        switch (vuMarkFound) {
            case RIGHT:
                if(team == RED) {
                    return -8;
                } else if(team == BLUE) {
                    return 8;
                }
            case CENTER:
                if(team == RED) {
                    return 0;
                } else if(team == BLUE) {
                    return 0;
                }
            case LEFT:
                if(team == RED) {
                    return 8;
                } else if(team == BLUE) {
                    return -8;
                }
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

    Team team;
    Position position;
}
