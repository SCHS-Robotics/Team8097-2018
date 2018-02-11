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

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="Blue Autonomous", group ="Autonomous")
public class BlueAutonomous extends Autonomous {
    RelicRecoveryVuMark vuMark = null;

    public void runOpMode() {
        ElapsedTime runtime = new ElapsedTime();
        team = Team.BLUE;
        position = Position.CLOSE;

        // Lazily added vuforia, add to each autonomous

        initialize();

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.vuforiaLicenseKey = "AWRsObH/////AAAAGU5bp4bnDkCYjwnKsD5okRCL7t6ejVuLHi3TwTkPTSo+EuLnlmB+G2Rz4GOel217l0cjjlYjJfot5pvsspqgEUJvtNDeoOacTA3bzKaeAFUoBeQA2r3VwolpdWR/6xxq9EraYiLIkOLee51c2Uqtzlvk8Qav301W2TJOdPbotZUAndR6QlIQ7m2UVZWY+2qlenB36jIF3ZGotK/QwihY0/96KWzHtbIPUheU4CiJmRlIi3xMGREt3SYgcPV3L/WMPi+WW7GSSoh9IVaVnfGmTZD2cWSGeB/x4RDHdUbePjZrEQ1OPNR/LvjbRYWkX+QgQUqmyff0/Etuf9o0oJ9PlYeeteZPv1m/2hiB/mUG9tz1";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        VuforiaLocalizer  vuforia = ClassFactory.createVuforiaLocalizer(parameters);
        VuforiaTrackables relicTrackables = vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate");
        RelicRecoveryVuMark vuMark = null;


        servoHorizontalHit.setPosition(HORIZONTAL_AUTO_START_POS);
        servoVerticalHit.setPosition(VERTICAL_AUTO_START_POS);
        servoTopLeftGrab.setPosition(1);
        servoTopRightGrab.setPosition(0);
        servoBottomLeftGrab.setPosition(1);
        servoBottomRightGrab.setPosition(0);
        relicTrackables.activate();

        runtime.reset();
        resetEncoders(motorBL, motorBR, motorFL, motorFR, motorLeftLift, motorRightLift);
        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            motorLeftLift.setPower(0.15);
            motorRightLift.setPower(-0.15);
            sleep(1000);
            motorRightLift.setPower(0);
            motorLeftLift.setPower(0);
            double vuTimer = runtime.time();
            do {
                vuMark = RelicRecoveryVuMark.from(relicTemplate);
                if (runtime.time() - vuTimer >= 5) {
                    vuMark = RelicRecoveryVuMark.CENTER;
                    break;
                }
            } while(vuMark == RelicRecoveryVuMark.UNKNOWN);
            telemetry.addData("VuMark", vuMark = RelicRecoveryVuMark.from(relicTemplate));
            telemetry.update();

            hitJewel();
            sleep(500);
            servoVerticalHit.setPosition(VERTICAL_AUTO_START_POS);
            servoHorizontalHit.setPosition(HORIZONTAL_AUTO_START_POS);

            moveToCrypto();
            alignToCrypto(targetColumnDistance(vuMark));
            break;
        }
    }
}
