/* Copyright (c) 2021 FIRST. All rights reserved.
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
import com.arcrobotics.ftclib.gamepad.ToggleButtonReader;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This file contains an example of a Linear "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop
 * period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When a selection is made from the menu, the corresponding OpMode is executed.
 * <p>
 * This particular OpMode illustrates driving a 4-motor Omni-Directional (or
 * Holonomic) robot.
 * This code will work with either a Mecanum-Drive or an X-Drive train.
 * Both of these drives are illustrated at
 * https://gm0.org/en/latest/docs/robot-design/drivetrains/holonomic.html
 * Note that a Mecanum drive must display an X roller-pattern when viewed from
 * above.
 * <p>
 * Holonomic drives provide the ability for the robot to move in three axes
 * (directions) simultaneously.
 * Each motion axis is controlled by one Joystick axis.
 * <p>
 * 1) Axial: Driving forward and backwards Left-joystick Forward/Backwards
 * 2) Lateral: Strafing right and left Left-joystick Right and Left
 * 3) Yaw: Rotating Clockwise and counter clockwise Right-joystick Right and
 * Left
 * <p>
 * This code is written assuming that the right-side motors need to be reversed
 * for the robot to drive forward.
 * When you first test your robot, if it moves backwards when you push the left
 * stick forward, then you must flip
 * the direction of all 4 motors (see code below).
 */

@TeleOp(name = "Omni/Mecacum Test", group = "test")
//@Disabled
public class EdinaFTCOmniMecanumTest extends LinearOpMode {
    private final ElapsedTime runtime = new ElapsedTime();
    boolean testMode = false;
    GamepadEx gamePadEx;

    @Override
    public void runOpMode() {
        gamePadEx = new GamepadEx(gamepad1);
        ToggleButtonReader startToggle = new ToggleButtonReader(gamePadEx, GamepadKeys.Button.START);
        // Initialize the hardware variables. Note that the strings used here must
        // correspond
        // to the names assigned during the robot configuration step on the DS or RC
        // devices.
        DcMotorEx leftFrontDrive = hardwareMap.get(DcMotorEx.class, "leftfrontdrive");
        DcMotorEx leftBackDrive = hardwareMap.get(DcMotorEx.class, "leftbackdrive");
        DcMotorEx rightFrontDrive = hardwareMap.get(DcMotorEx.class, "rightfrontdrive");
        DcMotorEx rightBackDrive = hardwareMap.get(DcMotorEx.class, "rightbackdrive");

        // Most robots need the motors on one side to be reversed to drive forward.
        // When you first test your robot, push the left joystick forward
        // and flip the direction ( FORWARD <-> REVERSE ) of any wheel that runs
        // backwards
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.addData("", "Use the Start button to toggle test mode.");
        telemetry.addData("", "Use X, A, Y or B to run each motor forward.");
        telemetry.update();

        waitForStart();
        runtime.reset();
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
//            gamePadEx.readButtons();
            startToggle.readValue();
            double max;
            // Start toggles test mode to use x, a, y and b to test each motor.
            testMode = startToggle.getState();

            // POV Mode uses left joystick to go forward & strafe, and right joystick to
            // rotate.
            double axial = -gamepad1.left_stick_y; // Note: pushing stick forward gives negative value
            double lateral = gamepad1.left_stick_x;
            double yaw = gamepad1.right_stick_x;

            // Combine the joystick requests for each axis-motion to determine each wheel's
            // power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            double leftFrontPower = axial + lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;
            double leftBackPower = axial - lateral + yaw;
            double rightBackPower = axial + lateral - yaw;

            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            if (max > 1.0) {
                leftFrontPower /= max;
                rightFrontPower /= max;
                leftBackPower /= max;
                rightBackPower /= max;
            }

            // This is test code:
            //
            // Each button should make the corresponding motor run FORWARD.
            // 1) First get all the motors to take to correct positions on the robot
            // by adjusting your Robot Configuration if necessary.
            // 2) Then make sure they run in the correct direction by modifying the
            // the setDirection() calls above.
            if (testMode) {
                leftFrontPower = gamepad1.x ? 1.0 : 0.0; // X gamepad
                leftBackPower = gamepad1.a ? 1.0 : 0.0; // A gamepad
                rightFrontPower = gamepad1.y ? 1.0 : 0.0; // Y gamepad
                rightBackPower = gamepad1.b ? 1.0 : 0.0; // B gamepad
            }

            // Send calculated power to wheels
            leftFrontDrive.setPower(leftFrontPower);
            rightFrontDrive.setPower(rightFrontPower);
            leftBackDrive.setPower(leftBackPower);
            rightBackDrive.setPower(rightBackPower);

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime);
            telemetry.addData("Test Mode", testMode);
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
            telemetry.addData("Front Ticks Left/right", "%d, %d", leftFrontDrive.getCurrentPosition(), rightFrontDrive.getCurrentPosition());
            telemetry.addData("Back Ticks Left/right", "%d, %d", leftBackDrive.getCurrentPosition(), rightBackDrive.getCurrentPosition());
            telemetry.update();
        }
    }
}
