package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Hardware {
    HardwareMap hwMap;
    private ElapsedTime runtime = new ElapsedTime();

    //General
    public final int pulsesPerRevolution = 1120;
    public final int encoderLeeway = 100;
    public final double triggerLeeway = 0.1;
    public final double motorPowerLeeway = 0.1;

    //Drive Train
    public DcMotor  FL, BL, FR, BR;
    ModernRoboticsI2cGyro gyro;
    public final int wheelDiameter = 4;
    public final int gyroLeeway = 5;

    //Collecting
    public DcMotor collection;
    public final double collectionSpeed = 1;

    //Shooting
    public DcMotor popper;
    Servo ballRelease;
    public final double popperSpeed = 0.8;
    public final double ballReleaseDownPosition = 0.74;
    public final double ballReleaseUpPosition = 0.14;

    //Beacon Pushing
    CRServo beaconPusher;
    OpticalDistanceSensor odsLeft;
    OpticalDistanceSensor odsRight;
    ColorSensor colorSensor;
    public final double beaconPushingSpeed = 1;
    public final double leftODSWhiteRaw = 0.15;
    public final double rightODSWhiteRaw = 0.2;

    public void init(HardwareMap ahwMap) {
        hwMap = ahwMap;

        //Drive Train
        FL = hwMap.dcMotor.get("FL");
        BL = hwMap.dcMotor.get("BL");
        FR = hwMap.dcMotor.get("FR");
        BR = hwMap.dcMotor.get("BR");
        FL.setDirection(DcMotor.Direction.REVERSE);
        BL.setDirection(DcMotor.Direction.REVERSE);
        gyro = (ModernRoboticsI2cGyro) hwMap.gyroSensor.get("gyro");
        gyro.calibrate();

        //Collecting
        collection = hwMap.dcMotor.get("collection");

        //Shooting
        popper = hwMap.dcMotor.get("popper");
        ballRelease = hwMap.servo.get("ballrelease");
        popper.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        popper.setPower(popperSpeed);
        ballRelease.setPosition(ballReleaseUpPosition);

        //Beacon Pushing
        beaconPusher = hwMap.crservo.get("beaconpusher");
        odsLeft = hwMap.opticalDistanceSensor.get("odsleft");
        odsRight = hwMap.opticalDistanceSensor.get("odsright");
        colorSensor = hwMap.colorSensor.get("colorsensor");
        odsLeft.enableLed(true);
        odsRight.enableLed(true);
        colorSensor.enableLed(true);

        while (gyro.isCalibrating());
    }

    public void move (double speed) {
        setModes(DcMotor.RunMode.RUN_USING_ENCODER, FL, BL, FR, BR);

        FL.setPower(speed);
        BL.setPower(speed);
        FR.setPower(speed);
        BR.setPower(speed);
    }

    public void rotate (double speed) {
        setModes(DcMotor.RunMode.RUN_USING_ENCODER, FL, BL, FR, BR);

        FL.setPower(speed);
        BL.setPower(speed);
        FR.setPower(-speed);
        BR.setPower(-speed);
    }

    public void moveEncoder (double speed, double distance) {
        setModes(DcMotor.RunMode.RUN_TO_POSITION, FL, BL, FR, BR);

        double revolutions = distance / (wheelDiameter * Math.PI);
        int ticks = (int) (pulsesPerRevolution * revolutions);

        FL.setPower(speed);
        BL.setPower(speed);
        FR.setPower(speed);
        BR.setPower(speed);

        int newFLPosition = FL.getCurrentPosition() + ticks;
        int newBLPosition = BL.getCurrentPosition() + ticks;
        int newFRPosition = FR.getCurrentPosition() + ticks;
        int newBRPosition = BR.getCurrentPosition() + ticks;

        FL.setTargetPosition(newFLPosition);
        BL.setTargetPosition(newBLPosition);
        FR.setTargetPosition(newFRPosition);
        BR.setTargetPosition(newBRPosition);

        while(Math.abs(FL.getCurrentPosition() - newFLPosition) > encoderLeeway ||
              Math.abs(BL.getCurrentPosition() - newBLPosition) > encoderLeeway ||
              Math.abs(FR.getCurrentPosition() - newFRPosition) > encoderLeeway ||
              Math.abs(BR.getCurrentPosition() - newBRPosition) > encoderLeeway);

        //while(FL.isBusy() || BL.isBusy() || FR.isBusy() || BR.isBusy()); This is more beautiful

        sleep(1000);
    }

    public void stopGyro(int degrees) {
        int newOrientation = gyro.getHeading() + degrees;
        if (newOrientation > 359) {
            newOrientation -= 360;
        } else if (newOrientation < 0) {
            newOrientation += 360;
        }

        while (Math.abs(gyro.getHeading() - newOrientation) > gyroLeeway);

        move(0);
        sleep(1000);
    }

    public void stopLine() {
        while (odsLeft.getRawLightDetected() < leftODSWhiteRaw || odsRight.getRawLightDetected() < rightODSWhiteRaw);
        if (odsLeft.getRawLightDetected() < leftODSWhiteRaw) {
            FR.setPower(0);
            BR.setPower(0);
            while (odsLeft.getRawLightDetected() < leftODSWhiteRaw);
            FL.setPower(0);
            BL.setPower(0);
        } else if (odsRight.getRawLightDetected() < rightODSWhiteRaw) {
            FL.setPower(0);
            BL.setPower(0);
            while (odsRight.getRawLightDetected() < rightODSWhiteRaw);
            FR.setPower(0);
            BR.setPower(0);
        }
    }

    private void sleep(double milliseconds) {
        double initial = runtime.milliseconds();
        while (runtime.milliseconds() < initial + milliseconds);
    }

    public void setModes (DcMotor.RunMode runMode, DcMotor... motors) {
        for (DcMotor motor : motors) {
            motor.setMode(runMode);
        }
    }

    public void shootParticle() {
        int newPopperPosition = popper.getCurrentPosition() + pulsesPerRevolution;
        popper.setTargetPosition(newPopperPosition);
        while(Math.abs(popper.getCurrentPosition() - newPopperPosition) > encoderLeeway);
    }

    public void lowerBall() {
        ballRelease.setPosition(ballReleaseDownPosition);
        sleep(1000);
        ballRelease.setPosition(ballReleaseUpPosition);
        sleep(1000);
    }
}
