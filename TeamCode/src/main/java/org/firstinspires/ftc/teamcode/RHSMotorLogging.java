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

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/**
 * Demonstrates empty OpMode
 */
@TeleOp(name = "Motor Log", group = "test")
//@Disabled
public class RHSMotorLogging extends LinearOpMode {
    // Calculate the COUNTS_PER_INCH for your specific drive train.
    // Go to your motor vendor website to determine your motor's COUNTS_PER_MOTOR_REV
    // For external drive gearing, set DRIVE_GEAR_REDUCTION as needed.
    // For example, use a value of 2.0 for a 12-tooth spur gear driving a 24-tooth spur gear.
    // This is gearing DOWN for less speed and more torque.
    // For gearing UP, use a gear ratio less than 1.0. Note this will affect the direction of wheel rotation.
    static final double COUNTS_PER_MOTOR_REV = 384.5;   // eg: GoBILDA 435 RPM Yellow Jacket
    static final double MOTOR_RPM = 435;
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // No External Gearing.
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    // These constants define the desired driving/control characteristics
    // They can/should be tweaked to suit the specific robot drive train.
    static final double DRIVE_SPEED = 0.4;     // Max driving speed for better distance accuracy.
    static final double TURN_SPEED = 0.2;     // Max Turn speed to limit turn rate
    static final double HEADING_THRESHOLD = 1.0;    // How close must the heading get to the target before moving to next step.
    // Requiring more accuracy (a smaller number) will often make the turn take longer to get into the final position.
    // Define the Proportional control coefficient (or GAIN) for "heading control".
    // We define one value when Turning (larger errors), and the other is used when Driving straight (smaller errors).
    // Increase these numbers if the heading does not corrects strongly enough (eg: a heavy robot or using tracks)
    // Decrease these numbers if the heading does not settle on the correct value (eg: very agile robot with omni wheels)
    Datalog datalog;
    private ElapsedTime runtime = new ElapsedTime();
    /* Declare OpMode members. */
    private DcMotorEx leftFrontDrive = null;
    private DcMotorEx rightFrontDrive = null;
    private MotorEx leftBackDrive = null;
    private DcMotorEx rightBackDrive = null;

    private double driveSpeed = 0;
    private double turnSpeed = 0;

    private int leftBackPosition = 0;
    private int rightBackPosition = 0;
    private int leftFrontPosition = 0;
    private int rightFrontPosition = 0;

    private double leftBackVelocity = 0;
    private double rightBackVelocity = 0;
    private double leftFrontVelocity = 0;
    private double rightFrontVelocity = 0;

    private boolean driveComplete = false;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        leftBackDrive = new MotorEx(hardwareMap, "leftbackdrive", Motor.GoBILDA.RPM_435);
        rightBackDrive = hardwareMap.get(DcMotorEx.class, "rightbackdrive");
        leftFrontDrive = hardwareMap.get(DcMotorEx.class, "leftfrontdrive");
        rightFrontDrive = hardwareMap.get(DcMotorEx.class, "rightfrontdrive");

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // When run, this OpMode should start both motors driving forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        leftFrontDrive.setDirection(DcMotorEx.Direction.REVERSE);
        leftBackDrive.setInverted(true);
        rightFrontDrive.setDirection(DcMotorEx.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotorEx.Direction.FORWARD);

        leftFrontDrive.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setRunMode(Motor.RunMode.RawPower);
        rightBackDrive.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        leftFrontDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        // Initialize the datalog
        // Note that the order in which we set datalog fields
        // does *not* matter! The order is configured inside
        // the Datalog class constructor.
        datalog = new Datalog("datalogMotor");

        waitForStart();

        runtime.reset();
        leftFrontDrive.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        leftBackDrive.setRunMode(Motor.RunMode.PositionControl);
        rightBackDrive.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        while (opModeIsActive() && !driveComplete) {
            driveStraight(DRIVE_SPEED, 480, 15);
            sleep(20);
        }
    }

    /**
     * Display the various control parameters while driving
     */
    private void sendTelemetry() {
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Positions LB:RB:LF:RF", "%d:%d:%d:%d", leftBackPosition, rightBackPosition, leftFrontPosition, rightFrontPosition);
        telemetry.addData("Wheel Velocities LB:RB:LF:RF", "%6.2f:%6.2f:%6.2f:%6.2f\n", leftBackVelocity, rightBackVelocity, leftFrontVelocity, rightFrontVelocity);
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
        int moveCounts = (int) (distance * COUNTS_PER_INCH);
        leftBackPosition = leftBackDrive.getCurrentPosition();
        rightBackPosition = rightBackDrive.getCurrentPosition();
        leftFrontPosition = leftFrontDrive.getCurrentPosition();
        rightFrontPosition = rightFrontDrive.getCurrentPosition();
        int leftBackTarget = leftBackPosition + moveCounts;
        int rightBackTarget = rightBackPosition + moveCounts;
        int leftFrontTarget = leftFrontPosition + moveCounts;
        int rightFrontTarget = rightFrontPosition + moveCounts;

        // Set Target FIRST, then turn on RUN_TO_POSITION
        leftBackDrive.setRunMode(Motor.RunMode.PositionControl);
        leftBackDrive.setPositionCoefficient(0.05);
        leftBackDrive.setTargetPosition(leftBackTarget);
        leftBackDrive.set(0);
        leftBackDrive.setPositionTolerance(13);
        rightBackDrive.setTargetPosition(rightBackTarget);
        leftFrontDrive.setTargetPosition(leftFrontTarget);
        rightFrontDrive.setTargetPosition(rightFrontTarget);

        rightBackDrive.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        leftFrontDrive.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        rightFrontDrive.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        // Set the required driving speed  (must be positive for RUN_TO_POSITION)
        // Start driving straight, and then enter the control loop
        maxDriveSpeed = Math.abs(maxDriveSpeed);
        moveRobot(maxDriveSpeed, 0);

        // keep looping while we are still active, and motors are running.
        while (!leftBackDrive.atTargetPosition() &&
                rightBackDrive.isBusy() &&
                leftFrontDrive.isBusy() &&
                rightFrontDrive.isBusy() &&
                driveTimer.time() < driveTime) {

            // if driving in reverse, the motor correction also needs to be reversed
            if (distance < 0)
                turnSpeed *= -1.0;

            moveRobot(driveSpeed, turnSpeed);

            // Display drive status for the driver.
            sendTelemetry();
            datalog.target.set(leftBackTarget);
            datalog.position.set(leftBackPosition);
            datalog.velocity.set(leftBackVelocity);
            datalog.writeLine();
        }

        // Stop all motion & Turn off RUN_TO_POSITION
        moveRobot(0, 0);
        leftBackDrive.setRunMode(Motor.RunMode.VelocityControl);
        rightBackDrive.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        leftFrontDrive.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        driveComplete = true;
    }

    /**
     * This method takes separate drive (fwd/rev) and turn (right/left) requests,
     * combines them, and applies the appropriate speed commands to the left and right wheel motors.
     *
     * @param drive forward motor speed
     * @param turn  clockwise turning motor speed.
     */
    public void moveRobot(double drive, double turn) {
        driveSpeed = drive;     // save this value as a class member so it can be used by telemetry.
        turnSpeed = turn;      // save this value as a class member so it can be used by telemetry.

        double leftSpeed = drive - turn;
        double rightSpeed = drive + turn;

        // Scale speeds down if either one exceeds +/- 1.0;
        double max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
        if (max > 1.0) {
            leftSpeed /= max;
            rightSpeed /= max;
        }

        double leftTargetVelocity = PowerToTPS(leftSpeed);
        double rightTargetVelocity = PowerToTPS(rightSpeed);

        leftBackDrive.set(leftSpeed);
        leftFrontDrive.setVelocity(leftTargetVelocity);
        rightBackDrive.setVelocity(rightTargetVelocity);
        rightFrontDrive.setVelocity(rightTargetVelocity);
        leftBackVelocity = leftBackDrive.getVelocity();
        leftBackPosition = leftBackDrive.getCurrentPosition();
        rightBackVelocity = rightBackDrive.getVelocity(AngleUnit.DEGREES);
        leftFrontVelocity = leftFrontDrive.getVelocity(AngleUnit.DEGREES);
        rightFrontVelocity = rightFrontDrive.getVelocity(AngleUnit.DEGREES);
    }

    /**
     * Convert power to velocity.
     * double TPS = (power/60) * COUNTS_PER_MOTOR_REV;
     *
     * @param power A motor power within -1 to 1.
     * @return Ticks per second for use in velocity.
     */
    private double PowerToTPS(double power) {
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
                            velocity
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
