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

import com.arcrobotics.ftclib.controller.wpilibcontroller.SimpleMotorFeedforward;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import java.util.List;

/**
 * Demonstrates empty OpMode
 */
@TeleOp(name = "FtcLib Motor Log", group = "test")
//@Disabled
public class RHSMotorLogging extends LinearOpMode {
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
    private final ElapsedTime runtime = new ElapsedTime();
    // These are set in init.
    double countsPerMotorRev = 2781;
    double motorRPM = 0;
    double countsPerInch = 0;
    Datalog datalog;
    private DcMotorEx leftBackMotor = null;
    private SimpleMotorFeedforward motorFeedForward;
    private double driveSpeed = 0;
    private int leftBackTarget = 0;
    private int leftBackPosition = 0;
    private double leftBackVelocity = 0;
    private double leftBackCurrent = 0;
    private double feedForwardCalculate = 0;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        leftBackMotor = hardwareMap.get(DcMotorEx.class, "leftbackdrive");
        leftBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        motorFeedForward = new SimpleMotorFeedforward(10, .8);

        countsPerInch = (countsPerMotorRev * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);

        // Important Step 1: Instantiate motor first and with MotoeEc or DCMotorEx.
        // Important Step 2: Get access to a list of Expansion Hub Modules to enable changing caching methods.
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        // Important Step 3: Set all Expansion hubs to use the AUTO Bulk Caching mode
        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        // Initialize the datalog
        // Note that the order in which we set datalog fields
        // does *not* matter! The order is configured inside
        // the Datalog class constructor.
        datalog = new Datalog("drivemotor");

        while (!opModeIsActive()) {
            telemetry.addData("Counts per Rev", "%6.2f", countsPerMotorRev);
            telemetry.addData("Max RPM", "%6.2f", motorRPM);
            telemetry.addData("Counts per Inch", "%6.2f", countsPerInch);
            telemetry.addData("Acceleration", "%6.2f", leftBackCurrent);
            telemetry.update();
        }

        waitForStart();
        leftBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        runtime.reset();

        driveStraight(DRIVE_SPEED, 24, 10);
        // Stay here to allow time to read display.
        while (opModeIsActive() && !isStopRequested()) {
        }
    }

    /**
     * Display the various control parameters while driving
     */
    private void sendTelemetry() {
        telemetry.addData("Status", "Run Time: " + runtime.seconds());
        telemetry.addData("Target Position", leftBackTarget);
        telemetry.addData("Position", "%d", leftBackPosition);
        telemetry.addData("Target Speed", "%6.2f", driveSpeed);
        telemetry.addData("Velocity", "%6.2f", leftBackVelocity);
        telemetry.addData("Current", "%6.2f\n", leftBackCurrent);
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
        leftBackPosition = leftBackMotor.getCurrentPosition();
        leftBackTarget = leftBackPosition + moveCounts;

        leftBackMotor.setTargetPosition(leftBackTarget);
        leftBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBackMotor.setVelocityPIDFCoefficients(1.26, 0.126, 0, 12.6);
        leftBackMotor.setPositionPIDFCoefficients(8);
        leftBackMotor.setVelocity(MAX_VELOCITY);

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

        while (leftBackMotor.isBusy() &&
                !isStopRequested()) {
            driveSpeed = leftSpeed;
            leftBackVelocity = leftBackMotor.getVelocity();
            feedForwardCalculate = motorFeedForward.calculate(leftBackVelocity);
            leftBackMotor.setPower(feedForwardCalculate);

            leftBackPosition = leftBackMotor.getCurrentPosition();
            leftBackCurrent = leftBackMotor.getCurrent(CurrentUnit.AMPS);

            // Log selected values
            datalog.target.set(leftBackTarget);
            datalog.position.set(leftBackPosition);
            datalog.velocity.set(leftBackVelocity);
            datalog.current.set(leftBackCurrent);
            datalog.feedforward.set(feedForwardCalculate);
            datalog.writeLine();
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
