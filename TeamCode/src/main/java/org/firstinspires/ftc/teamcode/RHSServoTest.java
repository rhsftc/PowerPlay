package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "FTCLib Servo", group = "test")
//@Disabled
public class RHSServoTest extends LinearOpMode {
    private final double MAX_ANGLE = 360;
    private final double MIN_ANGLE = 0;
    private SimpleServo servo;

    @Override
    public void runOpMode() throws InterruptedException {
        servo = new SimpleServo(hardwareMap, "servo1", MIN_ANGLE, MAX_ANGLE);
        servo.setRange(MIN_ANGLE, MAX_ANGLE, AngleUnit.DEGREES);
        telemetry.addData("Servo", servo.getDeviceType());
        telemetry.update();
        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.dpad_left) {
                servo.turnToAngle(270, AngleUnit.DEGREES);
            } else if (gamepad1.dpad_right) {
                servo.turnToAngle(90, AngleUnit.DEGREES);
            } else if (gamepad1.dpad_down) {
                servo.turnToAngle(5, AngleUnit.DEGREES);
            } else if (gamepad1.dpad_up) {
                servo.turnToAngle(355, AngleUnit.DEGREES);
            }
            telemetry.addData("Angle", servo.getAngle());
            telemetry.addData("Position", servo.getPosition());
            telemetry.update();
        }
    }
}
