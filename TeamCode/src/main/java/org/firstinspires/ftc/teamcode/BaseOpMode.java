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
import org.firstinspires.ftc.robotcontroller.external.samples.SensorREVColorDistance;
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

    Servo servoLeftGrab;
    Servo servoRightGrab;
    Servo servoHorizontalHit;
    Servo servoVerticalHit;

    DcMotor motorBL;
    DcMotor motorBR;
    DcMotor motorFL;
    DcMotor motorFR;
    DcMotor motorLeftLift;
    DcMotor motorRightLift;

    BNO055IMU imu;

    ColorSensor colorSensorArm;

    // State used for updating telemetry
    Orientation angles;
    double heading;

    final double VERTICAL_AUTO_START_POS = .031;
    final double HORIZONTAL_AUTO_START_POS = .46;
    final double VERTICAL_TELEOP_START_POS = .576;
    final double HORIZONTAL_TELEOP_START_POS = .404;
    final double VERTICAL_END_POS = 1.0;
    final double HORIZONTAL_END_POS = .404;
    final double HORIZONTAL_RIGHT_END_POS = .537;
    final double HORIZONTAL_LEFT_END_POS = .271;

    // TODO: Keep getting these over and over literally every time someone does something on hardware.
    final double TICKS_PER_INCH = 100;
    final double TICKS_PER_INCH_SIDE = 150;

    // Thing to compensate for imbalance, experimental value.
    // It sure would be nice if I DIDN'T HAVE TO INCLUDE THIS, BUT SURELY IT WOULD BE TOO BIG OF A PROBLEM FOR HARDWARE TO FIX, NOW WOULDN'T IT
    final double DRIVE_WEIGHT_SCALAR = .68;

    final Scalar glyphBrownHSV = new Scalar(7.5, 50, 147.5);
    final Scalar glyphBrownColorRadius = new Scalar(7.5, 105, 107.5);
    final Scalar glyphGrayHSV = new Scalar(90, 6.5, 146);
    final Scalar glyphGrayColorRadius = new Scalar(90, 6.5, 73);
    final Scalar redHSV = new Scalar(3.5, 233, 162.5);
    final Scalar redColorRadius = new Scalar(3.5, 22, 92.5);
    final Scalar blueHSV = new Scalar(200, 200, 154);
    final Scalar blueColorRadius = new Scalar(20, 55, 110);

    protected String                detectColor;
    protected boolean               mIsColorSelected = false;
    protected Mat                   mRgba;
    protected Scalar                mBlobColorRgba;
    protected Scalar                mBlobColorHsv;
    protected Scalar                mBlobColorRadius;
    protected ColorBlobDetector     mDetector;
    protected Mat                   mSpectrum;
    protected Size                  SPECTRUM_SIZE;
    protected Scalar                CONTOUR_COLOR;

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

        colorSensorArm = hardwareMap.colorSensor.get("colorSense");
        colorSensorArm.setI2cAddress(I2cAddr.create7bit(0x39));
        colorSensorArm.enableLed(false);

        servoLeftGrab = hardwareMap.servo.get("servoLeftGrab");
        servoRightGrab = hardwareMap.servo.get("servoRightGrab");
        servoHorizontalHit = hardwareMap.servo.get("servoHorizontalHit");
        servoVerticalHit = hardwareMap.servo.get("servoVerticalHit");

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        // Creating motors
        motorBL = hardwareMap.dcMotor.get("motorBackLeft");
        motorBR = hardwareMap.dcMotor.get("motorBackRight");
        motorFL = hardwareMap.dcMotor.get("motorFrontLeft");
        motorFR = hardwareMap.dcMotor.get("motorFrontRight");
        motorLeftLift = hardwareMap.dcMotor.get("motorLiftLeft");
        motorRightLift = hardwareMap.dcMotor.get("motorLiftRight");

        // Setting up encoders
        motorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLeftLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRightLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Testing, ignore this for now. Allows the motors to "coast" instead of active braking.
        motorBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void resetEncoders(DcMotor...motors) {
        for(DcMotor motor : motors){
            if(motor.getCurrentPosition() != 0){
                motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public void stopMotors(DcMotor...motors) {
        for(DcMotor motor : motors){
            motor.setPower(0);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    // Movement code
    public void turnRight(double speed) {
        motorBL.setPower(-speed);
        motorBR.setPower(-speed);
        motorFL.setPower(-speed * DRIVE_WEIGHT_SCALAR);
        motorFR.setPower(-speed * DRIVE_WEIGHT_SCALAR);
    }

    public void turnLeft(double speed) {
        motorBL.setPower(speed);
        motorBR.setPower(speed);
        motorFL.setPower(speed * DRIVE_WEIGHT_SCALAR);
        motorFR.setPower(speed * DRIVE_WEIGHT_SCALAR);
    }
    
    public void goForward(double speed) {
        motorBL.setPower(speed);
        motorBR.setPower(-speed);
        motorFL.setPower(speed * DRIVE_WEIGHT_SCALAR);
        motorFR.setPower(-speed * DRIVE_WEIGHT_SCALAR);
    }

    public void goBackward(double speed) {
        motorBL.setPower(-speed);
        motorBR.setPower(speed);
        motorFL.setPower(-speed * DRIVE_WEIGHT_SCALAR);
        motorFR.setPower(speed * DRIVE_WEIGHT_SCALAR);
    }

    public void goLeft(double speed) {
        motorBL.setPower(-speed);
        motorBR.setPower(-speed);
        motorFL.setPower(speed * DRIVE_WEIGHT_SCALAR);
        motorFR.setPower(speed * DRIVE_WEIGHT_SCALAR);
    }

    public void goRight(double speed) {
        motorBL.setPower(speed);
        motorBR.setPower(speed);
        motorFL.setPower(-speed * DRIVE_WEIGHT_SCALAR);
        motorFR.setPower(-speed * DRIVE_WEIGHT_SCALAR);
    }

    public void goDiagonalForwardRight(double speed) {
        motorBL.setPower(speed);
        motorBR.setPower(0);
        motorFL.setPower(0);
        motorFR.setPower(-speed * DRIVE_WEIGHT_SCALAR);
    }

    public void goDiagonalForwardLeft(double speed) {
        motorBL.setPower(0);
        motorBR.setPower(-speed);
        motorFL.setPower(speed * DRIVE_WEIGHT_SCALAR);
        motorFR.setPower(0);
    }

    public void goDiagonalBackwardRight(double speed) {

        motorBL.setPower(0);
        motorBR.setPower(speed);
        motorFL.setPower(-speed * DRIVE_WEIGHT_SCALAR);
        motorFR.setPower(0);
    }

    public void goDiagonalBackwardLeft(double speed) {
        motorBL.setPower(-speed);
        motorBR.setPower(0);
        motorFL.setPower(0);
        motorFR.setPower(speed * DRIVE_WEIGHT_SCALAR);
    }

    public void goForwardDistance(double distance, double speed) throws InterruptedException{
        double targetPosition = -distance * TICKS_PER_INCH;
        resetEncoders(motorBL, motorBR, motorFL, motorFR);

        motorBL.setTargetPosition((int)targetPosition);
        motorFL.setTargetPosition((int)targetPosition);
        motorBR.setTargetPosition((int)-targetPosition);
        motorFR.setTargetPosition((int)-targetPosition);

        motorBL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motorBL.setPower(speed);
        motorBR.setPower(speed);
        motorFL.setPower(speed * DRIVE_WEIGHT_SCALAR);
        motorFR.setPower(speed * DRIVE_WEIGHT_SCALAR);

        while (motorBL.isBusy() && motorFR.isBusy() && motorBR.isBusy() && motorFL.isBusy()) {}

        stopMotors(motorBL, motorBR, motorFL, motorFR);
    }

    public void strafeLeftDistance (double distance, double speed) throws InterruptedException{
        double targetPosition = -distance * TICKS_PER_INCH_SIDE;
        resetEncoders(motorBL, motorBR, motorFL, motorFR);

        motorBL.setTargetPosition((int)targetPosition);
        motorFL.setTargetPosition((int)-targetPosition);
        motorBR.setTargetPosition((int)targetPosition);
        motorFR.setTargetPosition((int)-targetPosition);

        motorBL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motorBL.setPower(-speed);
        motorBR.setPower(-speed);

        double leftScalar = DRIVE_WEIGHT_SCALAR;
        double rightScalar = DRIVE_WEIGHT_SCALAR;
        motorFL.setPower(speed * leftScalar);
        motorFR.setPower(speed * rightScalar);

        double initialHeading = getHeading();

        while (motorBL.isBusy() && motorFR.isBusy() && motorBR.isBusy() && motorFL.isBusy()) {
            double currentHeading = getHeading();

            if (initialHeading > currentHeading) {
                rightScalar = DRIVE_WEIGHT_SCALAR + .15;
                leftScalar = DRIVE_WEIGHT_SCALAR - .15;

            } else if (initialHeading < currentHeading) {
                leftScalar = DRIVE_WEIGHT_SCALAR + .15;
                rightScalar = DRIVE_WEIGHT_SCALAR - .15;

            } else {
                leftScalar = DRIVE_WEIGHT_SCALAR;
                rightScalar = DRIVE_WEIGHT_SCALAR;
            }

            motorFL.setPower(speed * leftScalar);
            motorFR.setPower(speed * rightScalar);
            telemetry.addData("Initial Heading ", initialHeading);
            telemetry.addData("Current Heading ", getHeading());
            telemetry.update();
        }

        stopMotors(motorBL, motorBR, motorFL, motorFR);
    }

    public void strafeRightDistance (double distance, double speed) throws InterruptedException {
        double targetPosition = -distance * TICKS_PER_INCH_SIDE;
        resetEncoders(motorBL, motorBR, motorFL, motorFR);

        motorBL.setTargetPosition((int)-targetPosition);
        motorFL.setTargetPosition((int)targetPosition);
        motorBR.setTargetPosition((int)-targetPosition);
        motorFR.setTargetPosition((int)targetPosition);

        motorBL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motorBL.setPower(-speed);
        motorBR.setPower(-speed);
        double leftScalar = DRIVE_WEIGHT_SCALAR;
        double rightScalar = DRIVE_WEIGHT_SCALAR;
        motorFL.setPower(speed * leftScalar);
        motorFR.setPower(speed * rightScalar);

        double initialHeading = getHeading();

        while (motorBL.isBusy() && motorFR.isBusy() && motorBR.isBusy() && motorFL.isBusy()) {
            double currentHeading = getHeading();

            if (initialHeading > currentHeading) {
                leftScalar = DRIVE_WEIGHT_SCALAR + .15;
                rightScalar = DRIVE_WEIGHT_SCALAR - .15;

            } else if (initialHeading < currentHeading) {
                rightScalar = DRIVE_WEIGHT_SCALAR + .15;
                leftScalar = DRIVE_WEIGHT_SCALAR - .15;

            } else {
                leftScalar = DRIVE_WEIGHT_SCALAR;
                rightScalar = DRIVE_WEIGHT_SCALAR;
            }

            motorFL.setPower(speed * leftScalar);
            motorFR.setPower(speed * rightScalar);
            telemetry.addData("Initial Heading ", initialHeading);
            telemetry.addData("Current Heading ", getHeading());
            telemetry.update();
        }

        stopMotors(motorBL, motorBR, motorFL, motorFR);
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

        while (opModeIsActive() && Math.abs(getHeading() - angle) > tolerance) {
            if (getHeading() > angle) {
                turnRight(speed);
            } else {
                turnLeft(speed);
            }
            telemetry.addData("Heading", getHeading());
            telemetry.update();
        }
        stopMotors(motorBL, motorBR, motorFL, motorFR);
    }

    public void turnRightFromCurrent(double angle, double speed, double tolerance) {
        double turnAngle = getHeading() - Math.abs(angle);
        turnTo(turnAngle, speed, tolerance);
    }

    public void turnLeftFromCurrent(double angle, double speed, double tolerance) {
        double turnAngle = getHeading() + Math.abs(angle);
        turnTo(turnAngle, speed, tolerance);
    }

    public void toggleLift() throws InterruptedException{
        switch (liftState) {
            case DOWN:
                liftState = LiftState.UP;
                motorLeftLift.setTargetPosition(600);
                motorRightLift.setTargetPosition(-600);

                motorLeftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorRightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                motorLeftLift.setPower(-.20);
                motorRightLift.setPower(-.20);

                while (motorLeftLift.isBusy() && motorRightLift.isBusy()) {}

                motorRightLift.setPower(0);
                motorLeftLift.setPower(0);

                motorLeftLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                motorRightLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                break;

            case UP:
                liftState = LiftState.DOWN;
                motorLeftLift.setTargetPosition(0);
                motorRightLift.setTargetPosition(0);

                motorLeftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorRightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                motorLeftLift.setPower(.20);
                motorRightLift.setPower(.20);

                while (motorLeftLift.isBusy() && motorRightLift.isBusy()) {}

                motorRightLift.setPower(0);
                motorLeftLift.setPower(0);

                motorLeftLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                motorRightLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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

    public double getHeading() {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return angles.firstAngle;
    }

    enum LiftState {
        UP,
        DOWN
    }

    enum HitStatus {
        INITIAL,
        UP,
        DOWN
    }

    enum GrabStatus {
        OPEN,
        CLOSE,
        HALFOPEN
    }

    protected LiftState liftState;
    protected HitStatus hitStatus;
    protected GrabStatus grabStatus;

}

