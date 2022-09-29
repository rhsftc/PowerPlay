package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "FTCLib Servo Test", group = "Test")
//@Disabled
public class RHSServoTest extends LinearOpMode {
    private final double MAX_ANGLE = 360;
    private final double MIN_ANGLE = 0;
    private ServoEx servo;

    @Override
    public void runOpMode() throws InterruptedException {
        servo = new SimpleServo(hardwareMap, "servo1", MIN_ANGLE, MAX_ANGLE);

        while (opModeIsActive()){
            if (gamepad1.dpad_left) {
                servo.rotateByAngle(10);
            } else if (gamepad1.dpad_right) {
                servo.rotateByAngle(-10);
            }
        }
    }
}
