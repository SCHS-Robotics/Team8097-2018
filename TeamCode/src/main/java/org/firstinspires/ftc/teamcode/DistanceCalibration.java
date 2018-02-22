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
/* This OpMode is essentially a copy of the startPositioning() function in regular autonomous, only
to be used in the case that we're not allowed to move the robot after initialization. */
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.hardware.ServoControllerEx;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="Distance Calibration", group ="Autonomous")
public class DistanceCalibration extends Autonomous {
    public void runOpMode() {
        initialize();
        initializeTts();
        waitForStart();
        while (opModeIsActive()) {
            double distance = rangeSensor.getDistance(DistanceUnit.CM);
            if (distance > JEWEL_ARM_MAX_DISTANCE) {
                telemetry.addLine("Robot too far!");
            }
            else if (distance < JEWEL_ARM_MIN_DISTANCE) {
                telemetry.addLine("Robot too close!");
            } else {
                telemetry.addLine("Robot at optimal distance");
            }
            telemetry.addData("cm optical", "%.2f cm", rangeSensor.cmOptical());
            telemetry.addData("cm", "%.2f cm", rangeSensor.getDistance(DistanceUnit.CM));
            telemetry.addLine("Press A to accept positioning");
            telemetry.addLine("Press B to test Red team");
            telemetry.addLine("Press X to test Blue team");

            telemetry.update();
            if (gamepad1.a){
                break;
            }
            if (gamepad1.b){
                team = team.RED;
                hitJewel();
                telemetry.addLine("Restart to measure more");
                telemetry.update();
            }
            if (gamepad1.x){
                team = team.BLUE;
                hitJewel();
                telemetry.addLine("Restart to measure more");
                telemetry.update();
            }
        }
    }
}
