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
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Demonstrates empty OpMode
 */
@TeleOp(name = "Generic Tester", group = "test")
//@Disabled
public class RHSGenericTester extends OpMode {

    private ElapsedTime runtime = new ElapsedTime();
    ParkingPosition parkLocation = ParkingPosition.CENTER;
    ScoreJunction scoreJunction = ScoreJunction.NONE;
    StartPosition startPosition = StartPosition.LEFT;
    GamepadEx gamePadDrive;
    double distance = 0;
    double direction = 0;
    double TILE_SIZE = 24;

    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");
        gamePadDrive = new GamepadEx(gamepad1);

    }

    /*
     * Code to run when the op mode is first enabled goes here
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#start()
     */
    @Override
    public void init_loop() {
        gamePadDrive.readButtons();
        if (gamePadDrive.wasJustPressed(GamepadKeys.Button.X)) {
            startPosition = StartPosition.LEFT;
            telemetry.speak(startPosition.toString());
            telemetry.update();
        }

        if (gamePadDrive.wasJustPressed(GamepadKeys.Button.B)) {
            startPosition = StartPosition.RIGHT;
            telemetry.speak(startPosition.toString());
            telemetry.update();
        }

        if (gamePadDrive.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
            scoreJunction = scoreJunction.getNext();
            switch (scoreJunction) {
                case LOW:
                    telemetry.speak("Low");
                    break;
                case MEDIUM:
                    telemetry.speak("Medium");
                    break;
                case NONE:
                    telemetry.speak("None");
                    break;
            }
            telemetry.update();
        }

        if (gamePadDrive.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
            parkLocation = parkLocation.getNext();
            switch (parkLocation) {
                case LEFT:
                    telemetry.speak("Left");
                    break;
                case CENTER:
                    telemetry.speak("Center");
                    break;
                case RIGHT:
                    telemetry.speak("Right");
                    break;
            }
        }
        sendTelemetry();
    }

    /*
     * This method will be called ONCE when start is pressed
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#loop()
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * This method will be called repeatedly in a loop
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#loop()
     */
    @Override
    public void loop() {
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        double[] paths = getSignalZonePath();
        direction = paths[0];
        distance = paths[1];
        sendTelemetry();
    }

    /**
     * Get the distance and direction parameters for strafeRobot()
     * to park in the correct signal zone.
     *
     * @return double[] - [0] os the direction, [1] is the distance
     */
    public double[] getSignalZonePath() {
        double[] result;
        double direction = 0;
        double distance = 0;
        // No junction, robot is on the signal sleeve tape mark
        if (scoreJunction == ScoreJunction.NONE) {
            switch (parkLocation) {
                case LEFT:
                    direction = 270;
                    distance = TILE_SIZE;
                    break;
                case CENTER:
                    direction = 0;
                    distance = 0;
                    break;
                case RIGHT:
                    direction = 90;
                    distance = TILE_SIZE;
                    break;
            }
        }
        // Scoring medium junction, robot is centered on the tile in front of the medium junction
        if (scoreJunction == ScoreJunction.MEDIUM) {
            switch (startPosition) {
                case LEFT:
                    direction = 270;
                    switch (parkLocation) {
                        case LEFT:
                            distance = TILE_SIZE * 2.5;
                            break;
                        case CENTER:
                            distance = TILE_SIZE * 1.5;
                            break;
                        case RIGHT:
                            distance = TILE_SIZE * .5;
                            break;
                    }
                    break;
                case RIGHT:
                    direction = 90;
                    switch (parkLocation) {
                        case LEFT:
                            distance = TILE_SIZE * .5;
                            break;
                        case CENTER:
                            distance = TILE_SIZE * 1.5;
                            break;
                        case RIGHT:
                            distance = TILE_SIZE * 2.5;
                            break;
                    }
                    break;
            }
        }
        result = new double[]{direction, distance};
        return result;
    }

    public void sendTelemetry() {
        telemetry.addData("Start", startPosition);
        telemetry.addData("Junction", scoreJunction);
        telemetry.addData("Park", parkLocation);
        telemetry.addData("Distance", distance);
        telemetry.addData("Direction", direction);
        telemetry.update();
    }

    // Which junction to score cone (assume only 1 cone)
    public enum ScoreJunction {
        LOW,
        MEDIUM,
        NONE;

        public ScoreJunction getNext() {
            return values()[(ordinal() + 1) % values().length];
        }
    }

    public enum StartPosition {
        RIGHT,
        LEFT,
        NONE
    }

    public enum ParkingPosition {
        LEFT,
        CENTER,
        RIGHT;

        public ParkingPosition getNext() {
            return values()[(ordinal() + 1) % values().length];
        }
    }
}
