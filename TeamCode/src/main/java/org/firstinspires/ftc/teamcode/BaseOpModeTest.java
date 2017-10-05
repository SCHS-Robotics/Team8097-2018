package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.HashMap;

public abstract class BaseOpModeTest extends LinearOpMode {

    //decalaring "type" of variable to variable, doing this allows it to access the methods created
    //for it, ex: .setPositon for a servo
    ColorSensor colorSense;
    OpticalDistanceSensor rangeSense;

    Servo servo1;
    Servo servo2;
    Servo servo3;

    DcMotor motor1;
    DcMotor motor2;

    HashMap<DcMotor, Integer> encoderStartPos = new HashMap<>();

    //Setting constant variables, final so that it cannot be changed later by accident
    final double servoInitPosition = 0;

    int wheelEncoderPpr = 1680;


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
    public void initilaize()
    {
        //Assigning previously declared variables to expansion hub names
        colorSense = hardwareMap.colorSensor.get("colorMR");
        servo1 = hardwareMap.servo.get("servo1");
        servo2 = hardwareMap.servo.get("servo2");
        servo3 = hardwareMap.servo.get("servo3");
        rangeSense = hardwareMap.opticalDistanceSensor.get("rangeREV");

        //Creating motors
        motor2 = hardwareMap.dcMotor.get("motor2");
        motor1 = hardwareMap.dcMotor.get("motor1");

        //Setting up encoders
        motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //setting servo initial positions on initialize method
        servo1.setPosition(servoInitPosition);
        servo2.setPosition(servoInitPosition);
        servo3.setPosition(servoInitPosition);


        //rangeSensor = hardwareMap.get(DistanceSensor.class, "rangeREV");
        /*RANGE1 = hardwareMap.i2cDevice.get("rangeMR");
        RANGE1Reader = new I2cDeviceSynchImpl(RANGE1, RANGE1ADDRESS, false);*/

    }
    public double getCurrentRpm(int encoderPpr, DcMotor motor, int waitTime) {
        return ((double) (Math.abs(motor.getCurrentPosition()) - encoderStartPos.get(motor)) / encoderPpr) / (waitTime / 60000.0);
    }
}