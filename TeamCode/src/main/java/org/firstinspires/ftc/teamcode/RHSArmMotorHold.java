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

import com.arcrobotics.ftclib.controller.wpilibcontroller.ElevatorFeedforward;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Arm Hold", group = "test")
//@Disabled
public class RHSArmMotorHold extends LinearOpMode {
    static final double ARM_DRIVE_REDUCTION = .5;
    static final double ARM_WHEEL_DIAMETER_INCHES = 2.5;
    static final int LOW_JUNCTION = 14;
    static final int MEDIUM_JUNCTION = 24;
    static final int HIGH_JUNCTION = 34;
    static final int HOME_POSITION = 1;
    static final int CONE_HEIGHT = 5;
    static final int ADJUST_ARM_INCREMENT = 1;
    static final double MAX_POWER = 0.4;
    // Declare OpMode members.
    private ElapsedTime runTime;
    private MotorEx armMotor = null;
    private ElevatorFeedforward armFeedForward;
    private GamepadEx gamePadArm;
    private int armTarget = 0;
    private int armPosition = 0;
    private ArmPosition selectedPosition;
    private double armVelocity = 0;
    private double armCorrectedVelocity = 0;
    private double armAcceleration = 0;
    private boolean isAtTarget = false;
    // These are set in init.
    private double armCountsPerMotorRev = 0;
    private double armCountsPerInch = 0;

    @Override
    public void runOpMode() {
        runTime = new ElapsedTime();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        armMotor = new MotorEx(hardwareMap, "leftbackdrive", Motor.GoBILDA.RPM_435);
        armFeedForward = new ElevatorFeedforward(10, 20, 0);
        armCountsPerMotorRev = armMotor.ACHIEVABLE_MAX_TICKS_PER_SECOND;
        armCountsPerInch = ((armCountsPerMotorRev * ARM_DRIVE_REDUCTION) / (ARM_WHEEL_DIAMETER_INCHES * 3.145));
        armMotor.setInverted(false);
        armMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        gamePadArm = new GamepadEx(gamepad2);

        // Wait for start
        waitForStart();
        runTime.reset();
        armMotor.resetEncoder();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive() && !isStopRequested()) {
            gamePadArm.readButtons();
            ProcessArm();
//            sendTelemetry();
        }
    }

    public void sendTelemetry() {
        telemetry.addData("Run time", runTime);
        telemetry.addData("Selected Position", selectedPosition);
        telemetry.addData("Target Position", "%d", armTarget);
        telemetry.addData("Current Position", armPosition);
        telemetry.addData("Velocity", armVelocity);
        telemetry.addData("Corrected Velocity", armCorrectedVelocity);
        telemetry.addData("Acceleration", armAcceleration);
        telemetry.addData("At target", isAtTarget);
        telemetry.update();
    }

    public void ProcessArm() {
        ArmPosition position = null;
        // Adjust position
        if (gamePadArm.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
            position = ArmPosition.ADJUST_DOWN;
        }

        if (gamePadArm.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
            position = ArmPosition.ADJUST_UP;
        }

        // Low junction
        if (gamePadArm.wasJustPressed(GamepadKeys.Button.A)) {
            position = ArmPosition.LOW;
        }

        // medium junction
        if (gamePadArm.wasJustPressed(GamepadKeys.Button.B)) {
            position = ArmPosition.MEDIUM;
        }

        // high junction
        if (gamePadArm.wasJustPressed(GamepadKeys.Button.Y)) {
            position = ArmPosition.HIGH;
        }

        // ground junction
        if (gamePadArm.wasJustPressed(GamepadKeys.Button.X)) {
            position = ArmPosition.HOME;
        }

        if (position != null) {
            moveArm(position);
        }
    }


    public void moveArm(ArmPosition position) {
        selectedPosition = position;
        armPosition = armMotor.getCurrentPosition();
        switch (position) {
            case HOME:
                armTarget = HOME_POSITION * (int) armCountsPerInch;
                break;
            case LOW:
                armTarget = (LOW_JUNCTION * (int) armCountsPerInch);
                break;
            case MEDIUM:
                armTarget = (MEDIUM_JUNCTION * (int) armCountsPerInch);
                break;
            case HIGH:
                armTarget = (HIGH_JUNCTION * (int) armCountsPerInch);
                break;
            case ADJUST_UP:
                armTarget = armPosition + (ADJUST_ARM_INCREMENT * (int) armCountsPerInch);
                break;
            case ADJUST_DOWN:
                armTarget = armPosition - (ADJUST_ARM_INCREMENT * (int) armCountsPerInch);
                break;
            default:
                return;
        }

        // Prevent arm moving below HOME_POSITION
        armTarget = Math.max(armTarget, HOME_POSITION * (int) armCountsPerInch);
        armMotor.setTargetPosition(armTarget);
        armMotor.setPositionCoefficient(.003);
        armMotor.setPositionTolerance(25);
        armMotor.setTargetPosition(armTarget);
        armMotor.setRunMode(Motor.RunMode.PositionControl);

        while (!armMotor.atTargetPosition() && !isStopRequested()) {
            armMotor.set(MAX_POWER);
            armPosition = armMotor.getCurrentPosition();
            armVelocity = armMotor.getVelocity();
            armCorrectedVelocity = armMotor.getCorrectedVelocity();
            armAcceleration = armMotor.getAcceleration();
            isAtTarget = armMotor.atTargetPosition();
            sendTelemetry();
        }

//        armMotor.setVelocity(0);
//        armMotor.stopMotor();
    }

    public enum ArmPosition {
        HOME,
        LOW,
        MEDIUM,
        HIGH,
        ADJUST_UP,
        ADJUST_DOWN
    }
}
