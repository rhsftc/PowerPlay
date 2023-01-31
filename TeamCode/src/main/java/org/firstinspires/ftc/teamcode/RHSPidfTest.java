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

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.wpilibcontroller.ElevatorFeedforward;
import com.arcrobotics.ftclib.controller.wpilibcontroller.SimpleMotorFeedforward;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import java.util.List;

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
    static final double WHEEL_DIAMETER_INCHES = 4;     // For figuring circumference
    static final double DRIVE_SPEED = 0.4;     // Max driving speed for better distance accuracy.
    static final double MAX_VELOCITY = 2200;    // Use with feed forward.
    // Arm related
    static final double ARM_DRIVE_REDUCTION = 2;
    static final double ARM_WHEEL_DIAMETER_INCHES = 2.5;
    static final double ARM_MOTOR_RPM = 435;
    static final double ARM_COUNTS_PER_MOTOR_REV = 384.5;   // eg: GoBILDA 435 RPM Yellow Jacket
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
    double countsPerInch = 0;
    Datalog dataLog;
    private DcMotorEx motor = null;
    private SimpleMotorFeedforward motorFeedForward;
    GamepadEx gamePadArm;
    private int position = 0;
    private ArmPosition selectedPosition;
    private boolean isBusy = false;
    private double feedForwardCalculate = 0;
    private double driveSpeed = 0;
    private int target = 0;
    private int currentPosition = 0;
    private double velocity = 0;
    private double current = 0;
    private boolean isArmTest = false;

    @Override
    public void runOpMode() {
        boolean isMotorFinished = false;
        gamePadArm = new GamepadEx(gamepad2);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
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
            armFeedForward = new ElevatorFeedforward(12, 20, .9);
            dataLog = new Datalog("pidfarmvelocityposition");
        } else {
            motor = hardwareMap.get(DcMotorEx.class, "leftbackdrive");
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor.setDirection(DcMotorSimple.Direction.FORWARD);
            motorFeedForward = new SimpleMotorFeedforward(10, .8);
            countsPerInch = (countsPerMotorRev * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);
            dataLog = new Datalog("pidfvelocityposition");
        }

        // Important Step 1: Instantiate motor first and with MotoeEc or DCMotorEx.
        // Important Step 2: Get access to a list of Expansion Hub Modules to enable changing caching methods.
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        // Important Step 3: Set all Expansion hubs to use the AUTO Bulk Caching mode
        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        telemetry.addLine("Ready for Start");
        telemetry.update();

        waitForStart();
        runtime.reset();
        while (opModeIsActive() && !isStopRequested()) {
            if (isArmTest) {
                gamePadArm.readButtons();
                ProcessArm();
            } else {
                if (!isMotorFinished) {
                    motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    driveStraight(DRIVE_SPEED, 24, 10);
                    isMotorFinished = true;
                }
            }
        }
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
        this.position = armMotor.getCurrentPosition();
        switch (position) {
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
                target = this.position + (ADJUST_ARM_INCREMENT * (int) ARM_COUNTS_PER_INCH);
                break;
            case ADJUST_DOWN:
                target = this.position - (ADJUST_ARM_INCREMENT * (int) ARM_COUNTS_PER_INCH);
                break;
            default:
                return;
        }

        // Prevent arm moving below HOME_POSITION
        target = Math.max(target, HOME_POSITION * (int) ARM_COUNTS_PER_INCH);
        armMotor.setTargetPosition(target);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setVelocityPIDFCoefficients(1.26, 0.126, 0, 12.6);
        armMotor.setPositionPIDFCoefficients(5);
        armMotor.setTargetPositionTolerance(5);
//        armMotor.setVelocity(TPS);
        armMotor.setVelocity(MAX_VELOCITY);

        while (armMotor.isBusy() && !isStopRequested()) {
            velocity = armMotor.getVelocity();
            feedForwardCalculate = armFeedForward.calculate(velocity);
//            armMotor.setVelocity(feedForwardCalculate);
            armMotor.setVelocity(MAX_VELOCITY);
            this.position = armMotor.getCurrentPosition();
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


    /**
     * Display the various control parameters while driving
     */
    public void sendTelemetry() {
        telemetry.addData("Status", "Run Time: " + runtime.seconds());
        telemetry.addData("Target Position", target);
        telemetry.addData("Position", "%d", currentPosition);
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

        // Determine new target position, and pass to motor controller
        int moveCounts = (int) (distance * countsPerInch);
        currentPosition = motor.getCurrentPosition();
        target = currentPosition + moveCounts;

        motor.setTargetPosition(target);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setVelocityPIDFCoefficients(1.26, 0.126, 0, 12.6);
        motor.setPositionPIDFCoefficients(8);
        motor.setVelocity(MAX_VELOCITY);

        while (opModeIsActive() &&
                !isStopRequested() &&
                driveTimer.seconds() < driveTime) {
            moveRobot(maxDriveSpeed);
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

        while (motor.isBusy() &&
                !isStopRequested()) {
            driveSpeed = leftSpeed;
            velocity = motor.getVelocity();
            feedForwardCalculate = motorFeedForward.calculate(velocity);
//            motor.setVelocity(MAX_VELOCITY);
            motor.setPower(feedForwardCalculate);

            currentPosition = motor.getCurrentPosition();
            current = motor.getCurrent(CurrentUnit.AMPS);

            // Log selected values
            logData();
            sendTelemetry();
        }
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
