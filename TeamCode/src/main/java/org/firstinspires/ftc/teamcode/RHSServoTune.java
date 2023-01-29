/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Demonstrates empty OpMode
 */
@TeleOp(name = "Servo Tune", group = "test")
//@Disabled
public class RHSServoTune extends OpMode {
    SimpleServo servo;
    boolean isInverted = false;
    private final double MIN_RANGE = 0;
    private final double MAX_RANGE = 180;
    private double turnToMinAngle = 45;
    private double turnToMaxAngle = 90;
    private double turnToAngle = 0;
    private GamepadEx gamePadDrive;
    private Telemetry.Item teleInverted;
    private Telemetry.Item teleTurnToMinAngle;
    private Telemetry.Item teleTurnToMaxAngle;
    private Telemetry.Item teleServoAngle;
    private Telemetry.Item teleServoPosition;
    private Telemetry.Item teleServoRange;

    @Override
    public void init() {
        gamePadDrive = new GamepadEx(gamepad1);
        servo = new SimpleServo(hardwareMap, "servo1", MIN_RANGE, MAX_RANGE);
        servo.setInverted(isInverted);

        telemetry.setAutoClear(false);
        telemetry.addLine("Back: Toggle Inverted");
        telemetry.addLine("DPAD Left: Decrease min angle");
        telemetry.addLine("DPAD Right: Increase min angle");
        telemetry.addLine("DPAD Up: Increase max angle");
        telemetry.addLine("DPAD Down: Decrease max angle");
        telemetry.addLine("A: Turn to min angle");
        telemetry.addLine("B: Turn to max angle");

        teleInverted = telemetry.addData("Inverted", isInverted);
        teleTurnToMinAngle = telemetry.addData("Min Angle", turnToMinAngle);
        teleTurnToMaxAngle = telemetry.addData("Max Angle", turnToMaxAngle);
        teleServoAngle = telemetry.addData("Servo angle", servo.getAngle());
        teleServoPosition = telemetry.addData("Servo position", servo.getPosition());
        teleServoRange = telemetry.addData("Servo range", servo.getAngleRange());
//        telemetry.update();
    }

    @Override
    public void init_loop() {
    }

    /*
     * This method will be called ONCE when start is pressed
     */
    @Override
    public void start() {

    }

    /*
     * This method will be called repeatedly in a loop
     */
    @Override
    public void loop() {
        gamePadDrive.readButtons();
        if (gamePadDrive.wasJustPressed(GamepadKeys.Button.BACK)) {
            isInverted = !isInverted;
        }

        if (gamePadDrive.wasJustReleased(GamepadKeys.Button.DPAD_LEFT)) {
            if (turnToMinAngle > MIN_RANGE) {
                turnToMinAngle -= 1;
            }
        }

        if (gamePadDrive.wasJustReleased(GamepadKeys.Button.DPAD_RIGHT)) {
            if (turnToMinAngle < MAX_RANGE) {
                turnToMinAngle += 1;
            }
        }

        if (gamePadDrive.wasJustReleased(GamepadKeys.Button.DPAD_UP)) {
            if (turnToMaxAngle < MAX_RANGE) {
                turnToMaxAngle += 1;
            }
        }

        if (gamePadDrive.wasJustReleased(GamepadKeys.Button.DPAD_DOWN)) {
            if (turnToMaxAngle > MIN_RANGE) {
                turnToMaxAngle -= 1;
            }
        }

        if (gamePadDrive.wasJustReleased(GamepadKeys.Button.A)) {
            servo.turnToAngle(turnToMinAngle);
        }

        if (gamePadDrive.wasJustReleased(GamepadKeys.Button.B)) {
            servo.turnToAngle(turnToMaxAngle);
        }

        sendTelemetry();
    }

    @Override
    public void stop() {

    }

    public void sendTelemetry() {
        teleInverted.setValue(isInverted);
        teleTurnToMinAngle.setValue(turnToMinAngle);
        teleTurnToMaxAngle.setValue(turnToMaxAngle);
        teleServoAngle.setValue(servo.getAngle());
        teleServoPosition.setValue(servo.getPosition());
        teleServoRange.setValue(servo.getAngleRange());
    }
}
