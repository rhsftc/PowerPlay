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

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.wpilibcontroller.ElevatorFeedforward;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@Config
@TeleOp(name = "PIDF Test", group = "motor")
//@Disabled
public class RHSPidfTest extends LinearOpMode {
    // Calculate the COUNTS_PER_INCH for your specific drive train.
    // Go to your motor vendor website to determine your motor's COUNTS_PER_MOTOR_REV
    // For external drive gearing, set DRIVE_GEAR_REDUCTION as needed.
    // For example, use a value of 2.0 for a 12-tooth spur gear driving a 24-tooth spur gear.
    // This is gearing DOWN for less speed and more torque.
    // For gearing UP, use a gear ratio less than 1.0. Note this will affect the direction of wheel rotation.
    static final double DRIVE_GEAR_REDUCTION = 1;     // No External Gearing.
    static final double WHEEL_DIAMETER_INCHES = 3.778;     // For figuring circumference
    static final double DRIVE_SPEED = .6;     // Max driving speed for better distance accuracy.
    static final double MOTOR_RPM = 435;
    static double COUNTS_PER_MOTOR_REV = 383.6;
    static double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);

    static double MOTOR_TPS = ((MOTOR_RPM * .75) / 60) * COUNTS_PER_MOTOR_REV;
    static final double HEADING_THRESHOLD = .8;
    static final double MOTOR_POSITION_COEFFICIENT = 5;
    static final int MOTOR_POSITION_TARGET_TOLERANCE = 10;
    static final double MAX_VELOCITY = 2200;    // Use with arm feed forward.
    // Arm related
    static final double ARM_DRIVE_REDUCTION = 2;
    static final double ARM_WHEEL_DIAMETER_INCHES = 2.5;
    static final double ARM_MOTOR_RPM = 435;
    static final double ARM_COUNTS_PER_MOTOR_REV = 383.6;   // eg: GoBilda 435 RPM Yellow Jacket
    static final double ARM_COUNTS_PER_WHEEL_REV = (ARM_COUNTS_PER_MOTOR_REV * ARM_DRIVE_REDUCTION);
    static final double ARM_COUNTS_PER_INCH = ARM_COUNTS_PER_WHEEL_REV / (ARM_WHEEL_DIAMETER_INCHES * 3.1415);
    public static int LOW_JUNCTION = 14;
    public static int MEDIUM_JUNCTION = 24;
    public static int HIGH_JUNCTION = 34;
    public static int HOME_POSITION = 1;
    public static int ADJUST_ARM_INCREMENT = 1;
    private double TPS = ((ARM_MOTOR_RPM * .75) / 60) * ARM_COUNTS_PER_WHEEL_REV;
    private DcMotorEx armMotor = null;
    private ElevatorFeedforward armFeedForward;
    private final ElapsedTime runtime = new ElapsedTime();
    // These are set in init.
    double countsPerMotorRev = 2900;
    Datalog dataLog;
    private DcMotorEx motor = null;
    GamepadEx gamePadArm;
    private int position = 0;
    private ArmPosition selectedPosition;
    private boolean isBusy = false;
    private double feedForwardCalculate = 0;
    private double driveSpeed = 0;
    private int target = 0;
    private double velocity = 0;
    private double current = 0;
    private boolean isArmTest = false;

    @Override
    public void runOpMode() {
        boolean isMotorFinished = false;
        gamePadArm = new GamepadEx(gamepad2);
//        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        while (!isStarted() &&
                !gamePadArm.wasJustPressed(GamepadKeys.Button.START) &&
                !isStopRequested()) {
            gamePadArm.readButtons();
            if (gamePadArm.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) {
                isArmTest = true;
            }

            if (gamePadArm.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {
                isArmTest = false;
            }

            telemetry.addData("Status", "Initialized");
            telemetry.addLine("Left bumper = Arm, Right bumper = Motor");
            telemetry.addData("Arm test", isArmTest);
            telemetry.addLine("then press Start");
            telemetry.update();
        }

        if (isArmTest) {
            armMotor = hardwareMap.get(DcMotorEx.class, "leftbackdrive");
            armMotor.setDirection(DcMotorSimple.Direction.FORWARD);
            armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            armFeedForward = new ElevatorFeedforward(12, 20, 5);
            dataLog = new Datalog("pidfarmelevatorFF");
        } else {
            motor = hardwareMap.get(DcMotorEx.class, "leftbackdrive");
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor.setDirection(DcMotorSimple.Direction.FORWARD);
            dataLog = new Datalog("pidfpositionvelocity");
        }

        telemetry.addLine("Ready for Start");
        telemetry.update();

        waitForStart();
        runtime.reset();
        while (opModeIsActive() && !isStopRequested()) {
            if (isArmTest) {
                gamePadArm.readButtons();
                ProcessArm();
                sendTelemetry();
            } else {
                if (!isMotorFinished) {
                    driveStraight(DRIVE_SPEED, 36, 3);
                    sleep(750);
                    isMotorFinished = true;
                }
            }
        }
    }

    public void ProcessArm() {
        ArmPosition armPosition = null;
        // Adjust position
        if (gamePadArm.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
            armPosition = ArmPosition.ADJUST_DOWN;
        }

        if (gamePadArm.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
            armPosition = ArmPosition.ADJUST_UP;
        }

        // Low junction
        if (gamePadArm.wasJustPressed(GamepadKeys.Button.A)) {
            armPosition = ArmPosition.LOW;
        }

        // medium junction
        if (gamePadArm.wasJustPressed(GamepadKeys.Button.B)) {
            armPosition = ArmPosition.MEDIUM;
        }

        // high junction
        if (gamePadArm.wasJustPressed(GamepadKeys.Button.Y)) {
            armPosition = ArmPosition.HIGH;
        }

        // ground junction
        if (gamePadArm.wasJustPressed(GamepadKeys.Button.X)) {
            armPosition = ArmPosition.HOME;
        }

        if (armPosition != null) {
            moveArm(armPosition);
        }
    }

    public void moveArm(ArmPosition armPosition) {
        selectedPosition = armPosition;
        position = armMotor.getCurrentPosition();
        switch (armPosition) {
            case HOME:
                target = HOME_POSITION * (int) ARM_COUNTS_PER_INCH;
                break;
            case LOW:
                target = (LOW_JUNCTION * (int) ARM_COUNTS_PER_INCH);
                break;
            case MEDIUM:
                target = (MEDIUM_JUNCTION * (int) ARM_COUNTS_PER_INCH);
                break;
            case HIGH:
                target = (HIGH_JUNCTION * (int) ARM_COUNTS_PER_INCH);
                break;
            case ADJUST_UP:
                target = position + (ADJUST_ARM_INCREMENT * (int) ARM_COUNTS_PER_INCH);
                break;
            case ADJUST_DOWN:
                target = position - (ADJUST_ARM_INCREMENT * (int) ARM_COUNTS_PER_INCH);
                break;
            default:
                return;
        }

        // Prevent arm moving below HOME_POSITION
        target = Math.max(target, HOME_POSITION * (int) ARM_COUNTS_PER_INCH);
        target = Math.min(target, HIGH_JUNCTION * (int) ARM_COUNTS_PER_INCH);
        armMotor.setTargetPosition(target);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        armMotor.setVelocityPIDFCoefficients(1.3429, 0.13429, 0, 13.429);
//        armMotor.setPositionPIDFCoefficients(5);
        armMotor.setTargetPositionTolerance(10);
        armMotor.setVelocity(TPS);
//        armMotor.setVelocity(MAX_VELOCITY);

        while (armMotor.isBusy()) {
            velocity = armMotor.getVelocity();
            feedForwardCalculate = armFeedForward.calculate(velocity);
            armMotor.setVelocity(feedForwardCalculate);
            position = armMotor.getCurrentPosition();
            current = armMotor.getCurrent(CurrentUnit.AMPS);
            isBusy = armMotor.isBusy();
            logData();
            sendTelemetry();
        }
    }

    public void logData() {
        dataLog.target.set(target);
        dataLog.position.set(position);
        dataLog.velocity.set(velocity);
        dataLog.current.set(current);
        dataLog.feedforward.set(feedForwardCalculate);
        dataLog.writeLine();
    }

    public enum ArmPosition {
        HOME,
        LOW,
        MEDIUM,
        HIGH,
        ADJUST_UP,
        ADJUST_DOWN
    }

    public void sendTelemetry() {
        telemetry.addData("Status", "Run Time: " + runtime.seconds());
        telemetry.addData("Target Position", target);
        telemetry.addData("Position", "%d", position);
        telemetry.addData("Target Speed", "%6.2f", driveSpeed);
        telemetry.addData("Velocity", "%6.2f", velocity);
        telemetry.addData("Current", "%6.2f\n", current);
        telemetry.update();
    }

    /**
     * Method to drive in a straight line, based on encoder counts.
     * Move will stop if either of these conditions occur:
     * 1) Move gets to the desired position
     * 2) Driver stops the opmode running.
     * 3) Timeout.
     *
     * @param maxDriveSpeed MAX Speed for forward/rev motion (range 0 to +1.0) .
     * @param distance      Distance (in inches) to move from current position.  Negative distance means move backward.
     */
    public void driveStraight(double maxDriveSpeed,
                              double distance,
                              double driveTime) {

        ElapsedTime driveTimer = new ElapsedTime();
        driveTimer.reset();

        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // Determine new target position, and pass to motor controller
        int moveCounts = (int) (distance * COUNTS_PER_INCH);
        position = motor.getCurrentPosition();
        target = position + moveCounts;

        motor.setTargetPosition(target);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setVelocityPIDFCoefficients(1.137, 0.1137, 0, 11.37);
        motor.setPositionPIDFCoefficients(MOTOR_POSITION_COEFFICIENT);
        motor.setTargetPositionTolerance(MOTOR_POSITION_TARGET_TOLERANCE);

        maxDriveSpeed = Math.abs(maxDriveSpeed);
        moveRobot(maxDriveSpeed);

        while (opModeIsActive() &&
                !isStopRequested() &&
                motor.isBusy() &&
                driveTimer.seconds() < driveTime) {
            moveRobot(maxDriveSpeed);
            logData();
            sendTelemetry();
        }
    }

    /**
     * @param drive forward motor speed
     */
    public void moveRobot(double drive) {
        double leftSpeed = drive;
        // Scale speed down if exceeds +/- 1.0;
        double max = Math.abs(leftSpeed);
        if (max > 1.0) {
            leftSpeed /= max;
        }

        driveSpeed = leftSpeed;
        motor.setVelocity(powerToTPS(driveSpeed));

        velocity = motor.getVelocity();
        position = motor.getCurrentPosition();
        current = motor.getCurrent(CurrentUnit.AMPS);
    }

    /**
     * Convert power to velocity.
     * double TPS = (power/60) * COUNTS_PER_MOTOR_REV;
     * ! For drive motors, not arm.
     *
     * @param power A motor power within -1 to 1.
     * @return Ticks per second for use in velocity.
     */
    private double powerToTPS(double power) {
        return ((power * MOTOR_RPM) / 60) * COUNTS_PER_MOTOR_REV;
    }

    /*
     * This class encapsulates all the fields that will go into the datalog.
     */
    public static class Datalog {
        // The underlying datalogger object - it cares only about an array of loggable fields
        private final Datalogger datalogger;

        // These are all of the fields that we want in the datalog.
        // Note that order here is NOT important. The order is important in the setFields() call below
        public Datalogger.GenericField velocity = new Datalogger.GenericField("Velocity");
        public Datalogger.GenericField target = new Datalogger.GenericField("Target");
        public Datalogger.GenericField position = new Datalogger.GenericField("Position");
        public Datalogger.GenericField current = new Datalogger.GenericField("Current");
        public Datalogger.GenericField feedforward = new Datalogger.GenericField("FeedForward");

        public Datalog(String name) {
            // Build the underlying datalog object
            datalogger = new Datalogger.Builder()

                    // Pass through the filename
                    .setFilename(name)

                    // Request an automatic timestamp field
                    .setAutoTimestamp(Datalogger.AutoTimestamp.DECIMAL_SECONDS)

                    // Tell it about the fields we care to log.
                    // Note that order *IS* important here! The order in which we list
                    // the fields is the order in which they will appear in the log.
                    .setFields(
                            target,
                            position,
                            velocity,
                            current,
                            feedforward
                    )
                    .build();
        }

        // Tell the datalogger to gather the values of the fields
        // and write a new line in the log.
        public void writeLine() {
            datalogger.writeLine();
        }
    }
}
