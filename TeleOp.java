package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="Main", group="Competition")
public class TeleOp extends OpMode {
    Hardware robot = new Hardware();

    //Drive Train
    double leftSpeed;
    double rightSpeed;
    double tempRightSpeed;
    boolean slowWheels;
    boolean driveTrainFlipped = false;
    boolean flippedRecentlyPressed = false;

    //Collecting
    boolean collectionIn;
    boolean collectionOut;

    //Shooting
    boolean shoot = false;
    boolean ballReleaseUp = false;
    boolean ballReleaseDown = false;

    //Beacon Pushing
    boolean beaconPusherIn = false;
    boolean beaconPusherOut = false;

    @Override
    public void init() {
        robot.init(hardwareMap);
        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void loop() {
        //Drive Train Assignment
        leftSpeed = -gamepad1.left_stick_y;
        rightSpeed = -gamepad1.right_stick_y;
        slowWheels = gamepad1.left_bumper;
        if (gamepad1.a && !flippedRecentlyPressed) {
            flippedRecentlyPressed = true;
            driveTrainFlipped = !driveTrainFlipped;
        } else if (!gamepad1.a && flippedRecentlyPressed) {
            flippedRecentlyPressed = false;
        }

        //Collection Assignment
        collectionIn = gamepad2.right_bumper;
        collectionOut = gamepad2.right_trigger > robot.triggerLeeway;

        //Shooting Assignment
        if (gamepad2.left_bumper && !robot.popper.isBusy()) {
            shoot = true;
        } else {
            shoot = false;
        }
        if (gamepad2.y) {
            ballReleaseUp = true;
            ballReleaseDown = false;
        } else if (gamepad2.a) {
            ballReleaseUp = false;
            ballReleaseDown = true;
        }

        //Beacon Pushing Assignment
        if (gamepad2.dpad_up) {
            beaconPusherOut = true;
            beaconPusherIn = false;
        } else if (gamepad2.dpad_down) {
            beaconPusherOut = false;
            beaconPusherIn = true;
        } else {
            beaconPusherOut = false;
            beaconPusherIn = false;
        }

        //---------------------------------------------

        //Drive Train Execution
        leftSpeed = Math.abs(leftSpeed) > robot.motorPowerLeeway ? leftSpeed : 0;
        rightSpeed = Math.abs(rightSpeed) > robot.motorPowerLeeway ? rightSpeed : 0;
        if (driveTrainFlipped) {
            tempRightSpeed = rightSpeed;
            rightSpeed = leftSpeed;
            leftSpeed = tempRightSpeed;
        } //gays not allowed
        robot.FL.setPower(leftSpeed);
        robot.BL.setPower(leftSpeed);
        robot.FR.setPower(rightSpeed);
        robot.BR.setPower(rightSpeed);

        //Collection Execution
        if (collectionIn) {
            robot.collection.setPower(robot.collectionSpeed);
        } else if (collectionOut) {
            robot.collection.setPower(-robot.collectionSpeed);
        } else {
            robot.collection.setPower(0);
        }

        //Shooting Execution
        if (shoot) {
            robot.popper.setTargetPosition(robot.popper.getCurrentPosition() + robot.pulsesPerRevolution);
        }
        if (ballReleaseUp) {
            robot.ballRelease.setPosition(robot.ballReleaseUpPosition);
        } else if (ballReleaseDown) {
            robot.ballRelease.setPosition(robot.ballReleaseDownPosition);
        }

        //Beacon Pushing Execution
        if (beaconPusherOut) {
            robot.beaconPusher.setPower(robot.beaconPushingSpeed);
        } else if (beaconPusherIn) {
            robot.beaconPusher.setPower(-robot.beaconPushingSpeed);
        } else {
            robot.beaconPusher.setPower(0);
        }
    }

}
