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

/**
 * Demonstrates empty OpMode
 */
@TeleOp(name = "Servo Tune", group = "test")
//@Disabled
public class RHSServoTune extends OpMode {
    SimpleServo servo;
    private final double MIN_RANGE = 0;
    private final double MAX_RANGE=180;
    private double minAngle = 0;
    private double maxAngle = 180;
    private double range = 0;
    private double turnToAngle = 0;
    private GamepadEx gamePadDrive;

    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");
        gamePadDrive = new GamepadEx(gamepad1);
        servo = new SimpleServo(hardwareMap, "servo1", minAngle, maxAngle);
        servo.setRange(minAngle, maxAngle);
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
        if (gamePadDrive.wasJustReleased(GamepadKeys.Button.DPAD_LEFT)) {
            if (minAngle > MIN_RANGE) {
                minAngle -= 1;
                updateRange();
            }
        }

        if (gamePadDrive.wasJustReleased(GamepadKeys.Button.DPAD_RIGHT)) {
            if (minAngle < MAX_RANGE) {
                minAngle += 1;
                updateRange();
            }
        }

        if (gamePadDrive.wasJustReleased(GamepadKeys.Button.DPAD_UP)) {
            if (maxAngle < MAX_RANGE) {
                maxAngle += 1;
                updateRange();
            }
        }

        if (gamePadDrive.wasJustReleased(GamepadKeys.Button.DPAD_DOWN)) {
            if (maxAngle > MIN_RANGE) {
                maxAngle -= 1;
                updateRange();
            }
        }

        if (gamePadDrive.wasJustReleased(GamepadKeys.Button.X)) {
            if (turnToAngle > MIN_RANGE) {
                turnToAngle -= 1;
                servo.turnToAngle(turnToAngle);
            }
        }

        if (gamePadDrive.wasJustReleased(GamepadKeys.Button.Y)) {
            if (turnToAngle < MAX_RANGE) {
                turnToAngle += 1;
                servo.turnToAngle(turnToAngle);
            }
        }

        if (gamePadDrive.wasJustReleased(GamepadKeys.Button.A)) {
            servo.turnToAngle(minAngle);
        }

        if (gamePadDrive.wasJustReleased(GamepadKeys.Button.B)) {
            servo.turnToAngle(maxAngle);
        }

        telemetry.addData("Min Angle", minAngle);
        telemetry.addData("Max Angle", maxAngle);
        telemetry.addData("Range", range);
        telemetry.addData("Turn to", turnToAngle);
        telemetry.addData("Servo angle", servo.getAngle());
        telemetry.addData("Servo position", servo.getPosition());
        telemetry.addData("Servo range", servo.getAngleRange());
        telemetry.update();
    }

    @Override
    public void stop() {

    }

    public void updateRange() {
        range = maxAngle - minAngle;
        servo.setRange(minAngle, maxAngle);
    }
}
