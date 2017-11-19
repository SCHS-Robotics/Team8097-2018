package org.firstinspires.ftc.teamcode;

import android.graphics.Camera;
import android.graphics.drawable.ScaleDrawable;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorMRRangeSensor;
import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;
import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.opencv.android.CameraBridgeViewBase;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;


import java.util.HashMap;
import java.util.Locale;

public abstract class BaseOpMode extends LinearOpMode implements CameraBridgeViewBase.CvCameraViewListener2{

    //decalaring "type" of variable to variable, doing this allows it to access the methods created
    //for it, ex: .setPositon for a servo

    Servo servoCamera;
    Servo servoLeftGrab;
    Servo servoRightGrab;

    DcMotor motorBL;
    DcMotor motorBR;
    DcMotor motorFL;
    DcMotor motorFR;
    DcMotor motorLift;

    BNO055IMU imu;

    // State used for updating telemetry
    Orientation angles;
    double heading;

    //HashMap<DcMotor, Integer> encoderStartPos = new HashMap<>();
    //Setting constant variables, final so that it cannot be changed later by accident
    final double servoCameraInitPosition = .267;
    final double TICKS_PER_CM_FORWARD = 53.6 / 1.5;
    final double INCHES_TO_CM = 2.54;
    final double TICKS_FOR_LIFT = (7 * 2.54) * TICKS_PER_CM_FORWARD;

    final double angleTolerance = 3;

    final Scalar glyphBrownHSV = new Scalar(7.5, 50, 147.5);
    final Scalar glyphBrownColorRadius = new Scalar(7.5, 105, 107.5);
    final Scalar glyphGrayHSV = new Scalar(90, 6.5, 146);
    final Scalar glyphGrayColorRadius = new Scalar(90, 6.5, 73);
    final Scalar redHSV = new Scalar(3.5, 233, 162.5);
    final Scalar redColorRadius = new Scalar(3.5, 22, 92.5);
    final Scalar blueHSV = new Scalar(147.3, 235.5, 95);
    final Scalar blueColorRadius = new Scalar(9.2, 19.5, 55);

    protected String detectColor;
    protected boolean              mIsColorSelected = false;
    protected Mat mRgba;
    protected Scalar               mBlobColorRgba;
    protected Scalar               mBlobColorHsv;
    protected Scalar               mBlobColorRadius;
    protected ColorBlobDetector    mDetector;
    protected Mat                  mSpectrum;
    protected Size                 SPECTRUM_SIZE;
    protected Scalar               CONTOUR_COLOR;


    //int wheelEncoderPpr = 1680;
    // Trying to get range sensor to work: -Includes changing I2C address
    //DistanceSensor rangeSensor;
    /*byte[] range1Cache; //The read will return an array of bytes. They are stored in this variable

    I2cAddr RANGE1ADDRESS = new I2cAddr(0x14); //Default I2C address for MR Range (7-bit)
    public static final int RANGE1_REG_START = 0x04; //Register to start reading
    public static final int RANGE1_READ_LENGTH = 2; //Number of byte to read

    public I2cDevice RANGE1;
    public I2cDeviceSynch RANGE1Reader;*/


    // Initialization, literally what happens when you select any OpMode and press "init"
    public void initialize()
    {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        //Assigning previously declared variables to expansion hub names

        // Setting up servos
        servoCamera = hardwareMap.servo.get("servoCamera");
        servoLeftGrab = hardwareMap.servo.get("servoLeftGrab");
        servoRightGrab = hardwareMap.servo.get("servoRightGrab");

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        // Creating motors
        motorBL = hardwareMap.dcMotor.get("motorBackLeft");
        motorBR = hardwareMap.dcMotor.get("motorBackRight");
        motorFL = hardwareMap.dcMotor.get("motorFrontLeft");
        motorFR = hardwareMap.dcMotor.get("motorFrontRight");
        motorLift = hardwareMap.dcMotor.get("motorLift");

        // Setting up encoders
        motorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // motorLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Testing, ignore this for now. Allows the motors to "coast" instead of active braking.
        motorBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motorBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motorFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motorFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // Setting servo initial positions on initialize method
        servoCamera.setPosition(servoCameraInitPosition);
        servoLeftGrab.setPosition(1);
        servoRightGrab.setPosition(0);
    }

    public void resetEncoders(DcMotor...motors) {
        for(DcMotor motor : motors){
            if(motor.getCurrentPosition() != 0){
                motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    // Movement code
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

    public void goForwardDistance(double distance, double speed) throws InterruptedException{
        resetEncoders(motorBL, motorBR, motorFL, motorFR);
        double totalEncoderTicks = distance * TICKS_PER_CM_FORWARD;
        goForward(speed);
        waitForEncoders(totalEncoderTicks);
        stop();
    }

    public void waitForEncoders(double encoderTicks) throws InterruptedException {
        while (getFurthestEncoder() < encoderTicks && opModeIsActive()) {
            sleep(1);
        }
    }

    public int getFurthestEncoder() {
        return Math.max(Math.max(Math.abs(motorBL.getCurrentPosition()), Math.abs(motorBR.getCurrentPosition())), Math.max(Math.abs(motorFL.getCurrentPosition()), Math.abs(motorFR.getCurrentPosition())));
    }

    public void turnTo(double angle, double speed, double tolerance) {
        double givenSpeed = speed;
        while(Math.abs(getHeading() - angle) > tolerance) {
            if (Math.abs(getHeading() - angle) < 40){
                if (Math.abs(getHeading() - angle) < 20) {
                    speed = 0.1;
                }
                else {
                    speed = 0.2;
                }
            }
            else {
                speed = givenSpeed;
            }

            if (getHeading() > angle) {
                turnRight(speed);
            } else {
                turnLeft(speed);
            }

            telemetry.addData("Heading", getHeading());
            telemetry.update();
        }
    }

    public void toggleLift(String direction, double ticks) throws InterruptedException{
        switch (direction) {
            case "up":
                motorLift.setPower(.25);
                waitForEncoders(ticks);
                break;
            case "down":
                motorLift.setPower(-.25);
                waitForEncoders(ticks);
                break;
        }
    }

    // OpenCV code
    public void setDetectColor(String newColor) {
        mIsColorSelected = false;
        switch (newColor) {
            case "brown":
                mBlobColorHsv = glyphBrownHSV;
                mBlobColorRadius = glyphBrownColorRadius;
                detectColor = "brown";
                break;
            case "gray":
                mBlobColorHsv = glyphGrayHSV;
                mBlobColorRadius = glyphGrayColorRadius;
                detectColor = "gray";
                break;
            case "red":
                mBlobColorHsv = redHSV;
                mBlobColorRadius = redColorRadius;
                detectColor = "red";
                break;
            case "blue":
                mBlobColorHsv = blueHSV;
                mBlobColorRadius = blueColorRadius;
                detectColor = "blue";
                break;
        }
        mBlobColorRgba = convertScalarHsv2Rgba(mBlobColorHsv);
        mDetector.setHsvColor(mBlobColorHsv);
        mDetector.setColorRadius(mBlobColorRadius);
        mIsColorSelected = true;
    }

    public double getHeading() {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        if (angles.firstAngle < 0){
            return angles.firstAngle += 360;
        }
        else{
            return angles.firstAngle;
        }
    }

    public Scalar convertScalarHsv2Rgba(Scalar hsvColor) {
        Mat pointMatRgba = new Mat();
        Mat pointMatHsv = new Mat(1, 1, CvType.CV_8UC3, hsvColor);
        Imgproc.cvtColor(pointMatHsv, pointMatRgba, Imgproc.COLOR_HSV2RGB_FULL, 4);

        return new Scalar(pointMatRgba.get(0, 0));
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

    void composeTelemetry() {

        telemetry.addAction(new Runnable() { @Override public void run() {
            angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            if(angles.firstAngle < 0){
                angles.firstAngle += 360;
                heading = angles.firstAngle + 360;
            } else {
                heading = angles.firstAngle;
            }
        }
        });

        telemetry.addLine()
                .addData("heading", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, getHeading());
                    }
                })
                .addData("heading", heading);
        }

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }


}

