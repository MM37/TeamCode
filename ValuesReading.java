package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="Values Reading", group="Testing")
public class ValuesReading extends OpMode {
    Hardware robot = new Hardware();

    @Override
    public void init() {
        robot.init(hardwareMap);
        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void loop() {
        telemetry.addData("Current Left", robot.odsLeft.getRawLightDetected());
        telemetry.addData("ODS Left Limit", robot.leftODSWhiteRaw);
        telemetry.addData("Current Right", robot.odsRight.getRawLightDetected());
        telemetry.addData("ODS Right Limit", robot.rightODSWhiteRaw);
    }

}
