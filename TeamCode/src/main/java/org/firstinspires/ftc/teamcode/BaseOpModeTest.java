package org.firstinspires.ftc.teamcode;

import android.graphics.Camera;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorMRRangeSensor;
import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;
import org.opencv.android.CameraBridgeViewBase;

import java.util.HashMap;

public abstract class BaseOpModeTest extends LinearOpMode implements CameraBridgeViewBase.CvCameraViewListener2{

    //decalaring "type" of variable to variable, doing this allows it to access the methods created
    //for it, ex: .setPositon for a servo
    ColorSensor colorSense;

    Servo servoCamera;
    Servo servoLeftGrab;
    Servo servoRightGrab;
    Servo servoRightHolder;
    Servo servoLeftHolder;

    DcMotor motorBL;
    DcMotor motorBR;
    DcMotor motorFL;
    DcMotor motorFR;


    //HashMap<DcMotor, Integer> encoderStartPos = new HashMap<>();
    //Setting constant variables, final so that it cannot be changed later by accident
    final double servoCameraInitPosition = .267;

    //int wheelEncoderPpr = 1680;


    // Trying to get range sensor to work: -Includes changing I2C address
    //DistanceSensor rangeSensor;
    /*byte[] range1Cache; //The read will return an array of bytes. They are stored in this variable

    I2cAddr RANGE1ADDRESS = new I2cAddr(0x14); //Default I2C address for MR Range (7-bit)
    public static final int RANGE1_REG_START = 0x04; //Register to start reading
    public static final int RANGE1_READ_LENGTH = 2; //Number of byte to read

    public I2cDevice RANGE1;
    public I2cDeviceSynch RANGE1Reader;*/


    //initialize() (method??) run when an op mode is run, this is where you map the config file names to the
    //variable names in the code
    public void initialize()
    {
        //Assigning previously declared variables to expansion hub names
        //colorSense =   hardwareMap.colorSensor.get("colorBottom");

        servoCamera = hardwareMap.servo.get("servoCamera");
        servoLeftGrab = hardwareMap.servo.get("servoLeftGrab");
        servoRightGrab = hardwareMap.servo.get("servoRightGrab");

        //Creating motors
        motorBL = hardwareMap.dcMotor.get("motorBackLeft");
        motorBR = hardwareMap.dcMotor.get("motorBackRight");
        motorFL = hardwareMap.dcMotor.get("motorFrontLeft");
        motorFR = hardwareMap.dcMotor.get("motorFrontRight");

        //Setting up encoders
        motorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //setting servo initial positions on initialize method
        servoCamera.setPosition(servoCameraInitPosition);
        servoLeftGrab.setPosition(1);
        servoRightGrab.setPosition(0);
    }

    //Movement code
    public void turnRight(double speed) {
        motorBL.setPower(-speed);
        motorBR.setPower(-speed);
        motorFL.setPower(-speed);
        motorFR.setPower(-speed);
    }

    public void turnLeft(double speed) {
        motorBL.setPower(speed);
        motorBR.setPower(speed);
        motorFL.setPower(speed);
        motorFR.setPower(speed);
    }
    
    public void goForward(double speed) {
        motorBL.setPower(speed);
        motorBR.setPower(-speed);
        motorFL.setPower(speed);
        motorFR.setPower(-speed);
    }

    public void goBackward(double speed) {
        motorBL.setPower(-speed);
        motorBR.setPower(speed);
        motorFL.setPower(-speed);
        motorFR.setPower(speed);
    }

    public void goLeft(double speed) {
        motorBL.setPower(-speed * 1);
        motorBR.setPower(-speed);
        motorFL.setPower(speed);
        motorFR.setPower(speed * 1);
    }

    public void goRight(double speed) {
        motorBL.setPower(speed * 1);
        motorBR.setPower(speed);
        motorFL.setPower(-speed);
        motorFR.setPower(-speed * 1);
    }

    public void goDiagonalForwardRight(double speed) {
        motorBL.setPower(speed);
        motorBR.setPower(0);
        motorFL.setPower(0);
        motorFR.setPower(-speed);
    }

    public void goDiagonalForwardLeft(double speed) {
        motorBL.setPower(0);
        motorBR.setPower(-speed);
        motorFL.setPower(speed);
        motorFR.setPower(0);
    }

    public void goDiagonalBackwardRight(double speed) {

        motorBL.setPower(0);
        motorBR.setPower(speed);
        motorFL.setPower(-speed);
        motorFR.setPower(0);
    }

    public void goDiagonalBackwardLeft(double speed) {
        motorBL.setPower(-speed);
        motorBR.setPower(0);
        motorFL.setPower(0);
        motorFR.setPower(speed);
    }

    public void startOpenCV(CameraBridgeViewBase.CvCameraViewListener2 cameraViewListener) {
        if (FtcRobotControllerActivity.mOpenCvCameraView.isEnabled())
            FtcRobotControllerActivity.mOpenCvCameraView.disableView();
        FtcRobotControllerActivity.turnOnCameraView.obtainMessage().sendToTarget();
        FtcRobotControllerActivity.mOpenCvCameraView.setCvCameraViewListener(cameraViewListener);
        FtcRobotControllerActivity.mOpenCvCameraView.enableView();
    }

    public void stopOpenCV() {
        FtcRobotControllerActivity.turnOffCameraView.obtainMessage().sendToTarget();
        FtcRobotControllerActivity.mOpenCvCameraView.disableView();
    }

   /* public double getCurrentRpm(int encoderPpr, DcMotor motor, int waitTime) {
        return ((double) (Math.abs(motor.getCurrentPosition()) - encoderStartPos.get(motor)) / encoderPpr) / (waitTime / 60000.0);
   */
}
