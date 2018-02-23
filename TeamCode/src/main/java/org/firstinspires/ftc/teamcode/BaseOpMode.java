package org.firstinspires.ftc.teamcode;

import android.speech.tts.TextToSpeech;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

import java.util.Locale;
import java.util.Random;

import static org.firstinspires.ftc.teamcode.BaseOpMode.Language.CHINESE;
import static org.firstinspires.ftc.teamcode.BaseOpMode.Language.ENGLISH;
import static org.firstinspires.ftc.teamcode.BaseOpMode.Language.GERMAN;
import static org.firstinspires.ftc.teamcode.BaseOpMode.Language.JAPANESE;
import static org.firstinspires.ftc.teamcode.BaseOpMode.Language.KOREAN;


public abstract class BaseOpMode extends LinearOpMode {

    //decalaring "type" of variable to variable, doing this allows it to access the methods created
    //for it, ex: .setPositon for a servo

    Servo servoTopLeftGrab;
    Servo servoTopRightGrab;
    Servo servoBottomLeftGrab;
    Servo servoBottomRightGrab;
    Servo servoHorizontalHit;
    Servo servoVerticalHit;

    DcMotor motorBL;
    DcMotor motorBR;
    DcMotor motorFL;
    DcMotor motorFR;
    DcMotor motorLeftLift;
    DcMotor motorRightLift;

    BNO055IMU imu;

    ColorSensor colorSensorLeft;
    ColorSensor colorSensorRight;

    ModernRoboticsI2cRangeSensor rangeSensor;

    TextToSpeech tts;
    Random generator = new Random();

    // State used for updating telemetry
    Orientation angles;
    double heading;

    // Servo values
    final double VERTICAL_AUTO_START_POS = 1;
    final double HORIZONTAL_AUTO_START_POS = .4;
    final double VERTICAL_TELEOP_START_POS = .576;
    final double HORIZONTAL_TELEOP_START_POS = .404;
    final double VERTICAL_END_POS = 0;
    final double HORIZONTAL_END_POS = .4;
    final double HORIZONTAL_RIGHT_END_POS = .6;
    final double HORIZONTAL_LEFT_END_POS = .2;

    // TODO: CHANGE THESE WHEN I CAN ACTUALLY SEE THE ROBOT AGAIN
    final double JEWEL_ARM_MAX_DISTANCE = 10;
    final double JEWEL_ARM_MIN_DISTANCE = 5;


    // TODO: Keep getting these over and over literally every time someone does something on hardware.
    final double TICKS_PER_INCH = 100;
    final double TICKS_PER_INCH_SIDE = 150;

    // TODO: Always make sure this is set, could be different every time hardware does something
    final double DRIVE_WEIGHT_SCALAR = .68;

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

        colorSensorLeft = hardwareMap.colorSensor.get("colorSenseLeft");
        colorSensorLeft.setI2cAddress(I2cAddr.create7bit(0x39));
        colorSensorLeft.enableLed(false);
        colorSensorRight = hardwareMap.colorSensor.get("colorSenseRight");
        colorSensorRight.setI2cAddress(I2cAddr.create7bit(0x40));
        colorSensorRight.enableLed(false);

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        rangeSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "rangeSensor");

        servoTopLeftGrab = hardwareMap.servo.get("servoTopLeftGrab");
        servoTopRightGrab = hardwareMap.servo.get("servoTopRightGrab");
        servoBottomLeftGrab = hardwareMap.servo.get("servoBottomLeftGrab");
        servoBottomRightGrab = hardwareMap.servo.get("servoBottomRightGrab");
        servoHorizontalHit = hardwareMap.servo.get("servoHorizontalHit");
        servoVerticalHit = hardwareMap.servo.get("servoVerticalHit");

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

        // Sets active braking instead of coast
        motorBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    void initializeTts() {
        tts = new TextToSpeech(hardwareMap.appContext, null);
        language = randomLanguage();
        updateTtsParams(language);
    }

    void updateTtsParams(Language newLanguage) {
        switch (newLanguage) {
            case JAPANESE:
                tts.setLanguage(Locale.JAPAN);
                tts.setPitch(1.5f);
                tts.setSpeechRate(1.5f);
            case KOREAN:
                tts.setLanguage(Locale.KOREA);
                tts.setPitch(1.5f);
                tts.setSpeechRate(1.5f);
            case CHINESE:
                tts.setLanguage(Locale.CHINA);
                tts.setPitch(1.5f);
                tts.setSpeechRate(1.5f);
            case ENGLISH:
                tts.setLanguage(Locale.UK);
                tts.setPitch(1f);
                tts.setSpeechRate(1.3f);
            case GERMAN:
                tts.setLanguage(Locale.GERMANY);
                tts.setPitch(1f);
                tts.setSpeechRate(1.5f);
        }
    }

    void ttsSpeak(String text) {
        tts.speak(text, TextToSpeech.QUEUE_FLUSH, null);
    }

    void resetEncoders(DcMotor...motors) {
        for(DcMotor motor : motors){
            if(motor.getCurrentPosition() != 0){
                motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    void stopMotors(DcMotor...motors) {
        for(DcMotor motor : motors){
            motor.setPower(0);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    // Movement code
    void turnRight(double speed) {
        motorBL.setPower(-speed);
        motorBR.setPower(-speed);
        motorFL.setPower(-speed * DRIVE_WEIGHT_SCALAR);
        motorFR.setPower(-speed * DRIVE_WEIGHT_SCALAR);
    }

    void turnLeft(double speed) {
        motorBL.setPower(speed);
        motorBR.setPower(speed);
        motorFL.setPower(speed * DRIVE_WEIGHT_SCALAR);
        motorFR.setPower(speed * DRIVE_WEIGHT_SCALAR);
    }
    
    void goForward(double speed) {
        motorBL.setPower(speed);
        motorBR.setPower(-speed);
        motorFL.setPower(speed * DRIVE_WEIGHT_SCALAR);
        motorFR.setPower(-speed * DRIVE_WEIGHT_SCALAR);
    }

    void goBackward(double speed) {
        motorBL.setPower(-speed);
        motorBR.setPower(speed);
        motorFL.setPower(-speed * DRIVE_WEIGHT_SCALAR);
        motorFR.setPower(speed * DRIVE_WEIGHT_SCALAR);
    }

    void goLeft(double speed) {
        motorBL.setPower(-speed);
        motorBR.setPower(-speed);
        motorFL.setPower(speed * DRIVE_WEIGHT_SCALAR);
        motorFR.setPower(speed * DRIVE_WEIGHT_SCALAR);
    }

    void goRight(double speed) {
        motorBL.setPower(speed);
        motorBR.setPower(speed);
        motorFL.setPower(-speed * DRIVE_WEIGHT_SCALAR);
        motorFR.setPower(-speed * DRIVE_WEIGHT_SCALAR);
    }

    void goDiagonalForwardRight(double speed) {
        motorBL.setPower(speed);
        motorBR.setPower(0);
        motorFL.setPower(0);
        motorFR.setPower(-speed * DRIVE_WEIGHT_SCALAR);
    }

    void goDiagonalForwardLeft(double speed) {
        motorBL.setPower(0);
        motorBR.setPower(-speed);
        motorFL.setPower(speed * DRIVE_WEIGHT_SCALAR);
        motorFR.setPower(0);
    }

    void goDiagonalBackwardRight(double speed) {

        motorBL.setPower(0);
        motorBR.setPower(speed);
        motorFL.setPower(-speed * DRIVE_WEIGHT_SCALAR);
        motorFR.setPower(0);
    }

    void goDiagonalBackwardLeft(double speed) {
        motorBL.setPower(-speed);
        motorBR.setPower(0);
        motorFL.setPower(0);
        motorFR.setPower(speed * DRIVE_WEIGHT_SCALAR);
    }

    void goForwardDistance(double distance, double speed) throws InterruptedException{
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

    void goBackwardDistance(double distance, double speed) throws InterruptedException{
        double targetPosition = -distance * TICKS_PER_INCH;
        resetEncoders(motorBL, motorBR, motorFL, motorFR);

        motorBL.setTargetPosition((int)-targetPosition);
        motorFL.setTargetPosition((int)-targetPosition);
        motorBR.setTargetPosition((int)targetPosition);
        motorFR.setTargetPosition((int)targetPosition);

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

    /* TODO: Consider getting rid of this because it's bloat
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
    */

    void turnTo(double angle, double speed, double tolerance) {
        double givenSpeed = speed;
        while (opModeIsActive() && Math.abs(getHeading() - angle) > tolerance) {
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
        stopMotors(motorBL, motorBR, motorFL, motorFR);
    }

    void turnRightFromCurrent(double angle, double speed, double tolerance) {
        double turnAngle = getHeading() - Math.abs(angle);
        turnTo(turnAngle, speed, tolerance);
    }

    void turnLeftFromCurrent(double angle, double speed, double tolerance) {
        double turnAngle = getHeading() + Math.abs(angle);
        turnTo(turnAngle, speed, tolerance);
    }

    void toggleLift() throws InterruptedException{
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

    private double getHeading() {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return angles.firstAngle;
    }

    Language randomLanguage() {
        int i = generator.nextInt(4) + 1;
        switch (i) {
            case 0:
                return JAPANESE;
            case 1:
                return KOREAN;
            case 2:
                return CHINESE;
            case 3:
                return ENGLISH;
            case 4:
                return GERMAN;
            default:
                return JAPANESE;
        }
    }

    void toggleLanguage() {
        switch (language) {
            case JAPANESE:
                language = KOREAN;
            case KOREAN:
                language = CHINESE;
            case CHINESE:
                language = ENGLISH;
            case ENGLISH:
                language = GERMAN;
            case GERMAN:
                language = JAPANESE;
        }
        updateTtsParams(language);
    }

    String welcomeText(){
        switch (language) {
            case JAPANESE:
                return "Kawaii neko robotto-chan is ready, senpai";
            case KOREAN:
                return "안녕하세요";
            case CHINESE:
                return "大家好。我们开始";
            case ENGLISH:
                return "";
            case GERMAN:
                return "Willkommen in unserem Panzerkampfwagen";
            default:
                return null;
        }
    }

    String vuMarkSpeech(RelicRecoveryVuMark vuMark) {
        switch (language) {
            case JAPANESE:
                return "Found vumark " + vuMark + " oni-chan";
            case KOREAN:
                return "나는 VuMark를 발견했다." + vuMark + "Unnie";
            case CHINESE:
                return "我看到VuMark。" + vuMark + "大哥";
            case ENGLISH:
                return "I've found the " + vuMark + "VuMark";
            case GERMAN:
                return "Ich habe das" + vuMark + "VuMark gefunden";
            default:
                return null;
        }
    }

    String[] randomLines(){
        switch (language) {
            case JAPANESE:
                return new String[] {"Nico Nico Ni", "Be u best"};
            case KOREAN:
                return new String[] {"감사합니다", "대박. 우리는 해냈다."};
            case CHINESE:
                return new String[] {"我想我们已经输了", "妈妈你看接了吗", "我们想修这个机器人"};
            case ENGLISH:
                return new String[] {"Speed and poweerrrr"};
            case GERMAN:
                return new String[] {"Geschwindigkeit und Kraft"};
            default:
                return null;
        }
    }

    String getRandomLine() {
        int i = generator.nextInt(3) + 1;
        return randomLines()[i];
    }

    enum Language {
        JAPANESE,
        KOREAN,
        CHINESE,
        ENGLISH,
        GERMAN,
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

    Language language;
    LiftState liftState;
    HitStatus hitStatus;
    GrabStatus grabStatus;

}

