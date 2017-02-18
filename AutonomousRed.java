package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;

@Autonomous(name="Red", group="Competition")
public class AutonomousRed extends LinearOpMode {

    Hardware robot = new Hardware();

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();

        //moves forward and shoot particle
        /*moveEncoder(-0.25, -12);//should be 12
        shootParticle();
        robot.lowerBall();
        shootParticle();

        //moves toward beacon
        robot.rotate(0.3);
        robot.stopGyro(125);
        //robot.moveEncoder(0.3, 66);*/

        robot.move(-0.1);
        stopLine();
    }

    public void stopLine() {
        while (robot.odsLeft.getRawLightDetected() < robot.leftODSWhiteRaw && robot.odsRight.getRawLightDetected() < robot.rightODSWhiteRaw) {
            telemetry.addData("Broken", "Never");
            telemetry.addData("Current Left", robot.odsLeft.getRawLightDetected());
            telemetry.addData("ODS Left Limit", robot.leftODSWhiteRaw);
            telemetry.addData("Current Right", robot.odsRight.getRawLightDetected());
            telemetry.addData("ODS Right Limit", robot.rightODSWhiteRaw);
            telemetry.update();
        }
        telemetry.addData("Broken", "1");
        telemetry.update();
        if (robot.odsLeft.getRawLightDetected() < robot.leftODSWhiteRaw) {
            robot.FR.setPower(0);
            robot.BR.setPower(0);
            while (robot.odsLeft.getRawLightDetected() < robot.leftODSWhiteRaw);
            robot.FL.setPower(0);
            robot.BL.setPower(0);
        } else if (robot.odsRight.getRawLightDetected() < robot.rightODSWhiteRaw) {
            robot.FL.setPower(0);
            robot.BL.setPower(0);
            while (robot.odsRight.getRawLightDetected() < robot.rightODSWhiteRaw);
            robot.FR.setPower(0);
            robot.BR.setPower(0);
        }
    }

    public void shootParticle() {
        int newPopperPosition = robot.popper.getCurrentPosition() + robot.pulsesPerRevolution;
        robot.popper.setTargetPosition(newPopperPosition);
        while(Math.abs(robot.popper.getCurrentPosition() - newPopperPosition) > robot.encoderLeeway) {
            telemetry.addData("Status", "Shoot Loop");
            telemetry.addData("Current", robot.popper.getCurrentPosition());
            telemetry.addData("new", newPopperPosition);
            telemetry.addData("Leeway", robot.encoderLeeway);
            telemetry.update();
        }
    }

    public void moveEncoder (double speed, double distance) {
        robot.setModes(DcMotor.RunMode.RUN_TO_POSITION, robot.FL, robot.BL, robot.FR, robot.BR);

        double revolutions = distance / (robot.wheelDiameter * Math.PI);
        int ticks = (int) (robot.pulsesPerRevolution * revolutions);

        robot.FL.setPower(speed);
        robot.BL.setPower(speed);
        robot.FR.setPower(speed);
        robot.BR.setPower(speed);

        int newFLPosition = robot.FL.getCurrentPosition() + ticks;
        int newBLPosition = robot.BL.getCurrentPosition() + ticks;
        int newFRPosition = robot.FR.getCurrentPosition() + ticks;
        int newBRPosition = robot.BR.getCurrentPosition() + ticks;

        robot.FL.setTargetPosition(newFLPosition);
        robot.BL.setTargetPosition(newBLPosition);
        robot.FR.setTargetPosition(newFRPosition);
        robot.BR.setTargetPosition(newBRPosition);

        while(Math.abs(robot.FL.getCurrentPosition() - newFLPosition) > robot.encoderLeeway ||
                Math.abs(robot.BL.getCurrentPosition() - newBLPosition) > robot.encoderLeeway ||
                Math.abs(robot.FR.getCurrentPosition() - newFRPosition) > robot.encoderLeeway ||
                Math.abs(robot.BR.getCurrentPosition() - newBRPosition) > robot.encoderLeeway) {
            telemetry.addData("Status", "Move Loop");
            telemetry.addData("Current", robot.FR.getCurrentPosition());
            telemetry.addData("New", newFRPosition);
            telemetry.addData("Current - New", robot.FR.getCurrentPosition() - newFRPosition);
            telemetry.addData("Leeway", robot.encoderLeeway);
            telemetry.addData("FL Binary", Math.abs(robot.FL.getCurrentPosition() - newFLPosition) > robot.encoderLeeway);
            telemetry.addData("BL Binary", Math.abs(robot.BL.getCurrentPosition() - newFLPosition) > robot.encoderLeeway);
            telemetry.addData("FR Binary", Math.abs(robot.FR.getCurrentPosition() - newFLPosition) > robot.encoderLeeway);
            telemetry.addData("BR Binary", Math.abs(robot.BR.getCurrentPosition() - newFLPosition) > robot.encoderLeeway);
            telemetry.update();
        }

        //while(FL.isBusy() || BL.isBusy() || FR.isBusy() || BR.isBusy()); This is more beautiful

        sleep(1000);
    }
}