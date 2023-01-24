package org.firstinspires.ftc.teamcode;


import com.arcrobotics.ftclib.controller.wpilibcontroller.ElevatorFeedforward;
import com.arcrobotics.ftclib.controller.wpilibcontroller.SimpleMotorFeedforward;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.List;

@Autonomous(name = "Bucket Auto", group = "autonomous")
public class RHSBucketAuto extends LinearOpMode {
    // Calculate the COUNTS_PER_INCH for your specific drive train.
    // Go to your motor vendor website to determine your motor's COUNTS_PER_MOTOR_REV
    // For external drive gearing, set DRIVE_GEAR_REDUCTION as needed.
    // For example, use a value of 2.0 for a 12-tooth spur gear driving a 24-tooth spur gear.
    // This is gearing DOWN for less speed and more torque.
    // For gearing UP, use a gear ratio less than 1.0. Note this will affect the direction of wheel rotation.
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // No External Gearing.
    static final double WHEEL_DIAMETER_INCHES = 3.778;     // For figuring circumference
    static final double DRIVE_SPEED = 0.4;     // Max driving speed for better distance accuracy.
    static final double TURN_SPEED = 0.4;
    // Define the Proportional control coefficient (or GAIN) for "heading control".
    // We define one value when Turning (larger errors), and the other is used when Driving straight (smaller errors).
    // Increase these numbers if the heading does not corrects strongly enough (eg: a heavy robot or using tracks)
    // Decrease these numbers if the heading does not settle on the correct value (eg: very agile robot with omni wheels)
    static final double P_TURN_GAIN = 0.01;     // Larger is more responsive, but also less stable
    static final double P_DRIVE_GAIN = 0.03;     // Larger is more responsive, but also less stable
    // How close must the heading get to the target before moving to next step.
    // Requiring more accuracy (a smaller number) will often make the turn take longer to get into the final position.
    static final double HEADING_THRESHOLD = 1.0;
    // These set the range for the gripper servo.
    static final double GRIPPER_MIN_ANGLE = 0;
    static final double GRIPPER_MAX_ANGLE = 45;
    // These set the open and close positions
    static final double GRIPPER_OPEN = 12;
    static final double GRIPPER_CLOSED = 23;

    // Arm related
    static final double ARM_DRIVE_REDUCTION = .5;
    static final double ARM_WHEEL_DIAMETER_INCHES = 2.5;
    static final int LOW_JUNCTION = 14;
    static final int MEDIUM_JUNCTION = 24;
    static final int HIGH_JUNCTION = 34;
    static final int HOME_POSITION = 1;
    static final int CONE_HEIGHT = 5;
    static final int ADJUST_ARM_INCREMENT = 1;
    static final double MAX_VELOCITY = 2200;
    private DcMotorEx armMotor = null;
    private ElevatorFeedforward armFeedForward;
    private int armTarget = 0;
    private int armPosition = 0;
    private RHSArmMotorHold.ArmPosition selectedPosition;
    private double armVelocity = 0;
    private double armCurrent = 0;
    private double feedForwardCalculate = 0;
    private boolean isBusy = false;

    private int pathSegment;
    private Datalog dataLog;
    private StartPosition startPosition = StartPosition.NONE;
    private int STRAFE_TIMEOUT = 3;     //Time to wait for strafing to finish.
    private SleeveDetection sleeveDetection;
    private OpenCvCamera camera;
    private IMU imu;
    private GamepadEx gamePadDrive;
    private String WEB_CAM_NAME = "webcam1";
    private SleeveDetection.ParkingPosition parkLocation;
    private MotorEx backLeftDrive;
    private MotorEx backRightDrive;
    private MotorEx frontLeftDrive;
    private MotorEx frontRightDrive;
    private MotorGroup leftMotors;
    private MotorGroup rightMotors;
    private MecanumDrive driveRobot;
    private SimpleMotorFeedforward simpleFeedForward;
    private SimpleServo gripperServo;

    private double countsPerMotorRev = 480;
    private double motorRPM = 300;
    private double countsPerInch = 0;
    private double armCountsPerMotorRev = 2781;
    private double armCountsPerInch = 0;

    private int backLeftTarget = 0;
    private int backRightTarget = 0;
    private int frontLeftTarget = 0;
    private int frontRightTarget = 0;
    private int backLeftPosition = 0;
    private int backRightPosition = 0;
    private int frontLeftPosition = 0;
    private int frontRightPosition = 0;

    private double turnSpeed = 0;
    private double driveSpeed = 0;
    private double leftSpeed = 0;
    private double rightSpeed = 0;
    private double headingError = 0;
    private double targetHeading = 0;
    private double robotHeading = 0;
    private double headingOffset = 0;

    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {
        dataLog = new Datalog("autolog");

        // Bulk reads
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        // Important: Set all Expansion hubs to use the AUTO Bulk Caching mode
        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.RIGHT;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.UP;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        // Now initialize the IMU with this mounting orientation
        // Note: if you choose two conflicting directions, this initialization will cause a code exception.
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        imu.resetYaw();

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, WEB_CAM_NAME), cameraMonitorViewId);
        sleeveDetection = new SleeveDetection();
        camera.setPipeline(sleeveDetection);

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(320, 240, OpenCvCameraRotation.SIDEWAYS_LEFT);
            }

            @Override
            public void onError(int errorCode) {
            }
        });

        gamePadDrive = new GamepadEx(gamepad1);

        armMotor = hardwareMap.get(DcMotorEx.class, "armmotor");
        armMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        armFeedForward = new ElevatorFeedforward(8, 20, .8);
        armCountsPerInch = ((armCountsPerMotorRev * ARM_DRIVE_REDUCTION) / (ARM_WHEEL_DIAMETER_INCHES * 3.145));

//TODO:        Use this for GoBilda motors.
//        countsPerMotorRev = backLeftDrive.ACHIEVABLE_MAX_TICKS_PER_SECOND;
//        motorRPM = backLeftDrive.getMaxRPM();
        countsPerInch = (countsPerMotorRev * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);

        simpleFeedForward = new SimpleMotorFeedforward(2, 10);
        backLeftDrive = new MotorEx(hardwareMap, "leftbackdrive", countsPerMotorRev, motorRPM);
        backRightDrive = new MotorEx(hardwareMap, "rightbackdrive", countsPerMotorRev, motorRPM);
        frontLeftDrive = new MotorEx(hardwareMap, "leftfrontdrive", countsPerMotorRev, motorRPM);
        frontRightDrive = new MotorEx(hardwareMap, "rightfrontdrive", countsPerMotorRev, motorRPM);
        backLeftDrive.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        frontLeftDrive.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        backLeftDrive.setInverted(true);
        frontLeftDrive.setInverted(true);
        backRightDrive.setInverted(false);
        frontRightDrive.setInverted(false);

        backLeftDrive.setPositionCoefficient(.05);
        frontLeftDrive.setPositionCoefficient(.05);
        backRightDrive.setPositionCoefficient(.05);
        frontRightDrive.setPositionCoefficient(.05);

        backLeftDrive.setPositionTolerance(10);
        frontLeftDrive.setPositionTolerance(10);
        backRightDrive.setPositionTolerance(10);
        frontRightDrive.setPositionTolerance(10);

        leftMotors = new MotorGroup(backLeftDrive, frontLeftDrive);
        rightMotors = new MotorGroup(backRightDrive, frontRightDrive);

        gripperServo = new SimpleServo(hardwareMap, "servo1", GRIPPER_MIN_ANGLE, GRIPPER_MAX_ANGLE);
        gripperServo.setInverted(true);
        openGripper();

        // Choose your start position.
        while (!isStarted() && !gamePadDrive.wasJustPressed(GamepadKeys.Button.START)) {
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

            telemetry.addLine("Select start position");
            telemetry.addLine("X=Left, B=Right, Start=adjust camera");
            telemetry.addData("Position: ", startPosition);
            telemetry.update();
        }

        // Adjust the camera here.
        while (!isStarted()) {
            parkLocation = getParkLocation();
            telemetry.addData("Start Position: ", startPosition);
            telemetry.addData("Park Location: ", parkLocation);
            telemetry.update();
        }

        waitForStart();
        pathSegment = 1;
        while (opModeIsActive() && !isStopRequested()) {
            switch (pathSegment) {
                case 1:
                    driveStraight(DRIVE_SPEED, 26, 0, 3);
                    sleep(750);
                    pathSegment = 2;
                    break;
                case 2:
                    // Where did the camera tell us to park?
                    switch (parkLocation) {
                        case LEFT:
                            strafeRobot(DRIVE_SPEED, 24, 270, STRAFE_TIMEOUT);
                            break;
                        case CENTER:
                            break;
                        case RIGHT:
                            strafeRobot(DRIVE_SPEED, 24, 90, STRAFE_TIMEOUT);
                            break;
                    }

                    sleep(750);
                    pathSegment = 4;
                    break;
                case 3:
                    turnToHeading(TURN_SPEED, -45, 3);
                    sleep(750);
                    pathSegment = 4;
                    break;
                case 4:
                    stopAllMotors();
//                TODO: Wait here so drive can read telemetry. Remove this after testing.
//                    while (!isStopRequested() && opModeIsActive()) {
//                        sendTelemetry();
//                    }

                    telemetry.addData("Status", "Path complete.");
                    telemetry.update();
                    break;
                default:
                    throw new IllegalStateException("Unexpected value: " + pathSegment);
            }
        }
    }

    /**
     * Get the result of the camera sleeve detection.
     */
    private SleeveDetection.ParkingPosition getParkLocation() {
        return sleeveDetection.getPosition();
    }
    /*
     * ====================================================================================================
     * Driving "Helper" functions are below this line.
     * These provide the high and low level methods that handle driving straight and turning.
     * ====================================================================================================
     */

    // **********  HIGH Level driving functions.  ********************

    /**
     * Method to drive in a straight line, on a fixed compass heading (angle), based on encoder counts.
     * Move will stop if either of these conditions occur:
     * 1) Move gets to the desired position
     * 2) Driver stops the opmode running.
     *
     * @param maxDriveSpeed MAX Speed for forward/rev motion (range 0 to +1.0) .
     * @param distance      Distance (in inches) to move from current position.
     * @param heading       Absolute Heading Angle (in Degrees) relative to last gyro reset.
     *                      0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                      If a relative angle is required, add/subtract from the current robotHeading.
     * @param driveTime     Time limit for drive operation. Seconds.
     */
    public void driveStraight(double maxDriveSpeed,
                              double distance,
                              double heading,
                              double driveTime) {

        ElapsedTime driveTimer = new ElapsedTime();
        driveTimer.reset();

        // Determine new target position, and pass to motor controller
        int moveCounts = (int) (distance * countsPerInch);
        getCurrentPositionsFromMotorGroups();
        backLeftTarget = backLeftPosition + moveCounts;
        backRightTarget = backRightPosition + moveCounts;
        frontLeftTarget = backLeftTarget + moveCounts;
        frontRightTarget = backRightTarget + moveCounts;

        // Set Target FIRST, then turn on RUN_TO_POSITION
        leftMotors.setTargetPosition(backLeftTarget);
        rightMotors.setTargetPosition(backRightTarget);

        leftMotors.setRunMode(Motor.RunMode.PositionControl);
        rightMotors.setRunMode(Motor.RunMode.PositionControl);

        // Set the required driving speed  (must be positive for RUN_TO_POSITION)
        // Start driving straight, and then enter the control loop
        maxDriveSpeed = Math.abs(maxDriveSpeed);
        moveRobot(maxDriveSpeed, 0);

        // keep looping while we are still active, and all motors are running.
        while (opModeIsActive() &&
                !isStopRequested() &&
                !(leftMotors.atTargetPosition() &&
                        rightMotors.atTargetPosition()) &&
                (driveTimer.time() < driveTime)) {

            // Determine required steering to keep on heading
            turnSpeed = getSteeringCorrection(heading, P_DRIVE_GAIN);

            // if driving in reverse, the motor correction also needs to be reversed
            if (distance < 0)
                turnSpeed *= -1.0;

            // Apply the turning correction to the current driving speed.
            moveRobot(maxDriveSpeed, turnSpeed);
        }

        // Stop all motion
        stopAllMotors();
    }

    /**
     * Method to spin on central axis to point in a new direction.
     * Move will stop if either of these conditions occur:
     * 1) Move gets to the heading (angle)
     * 2) Driver stops the opmode running.
     *
     * @param maxTurnSpeed Desired MAX speed of turn. (range 0 to +1.0)
     * @param heading      Absolute Heading Angle (in Degrees) relative to last gyro reset.
     *                     0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                     If a relative angle is required, add/subtract from current heading.
     * @param turnTime     Time limit (seconds) to complete turn.
     */
    public void turnToHeading(double maxTurnSpeed, double heading, double turnTime) {
        ElapsedTime turnTimer = new ElapsedTime();
        turnTimer.reset();

        // Run getSteeringCorrection() once to pre-calculate the current error
        getSteeringCorrection(heading, P_DRIVE_GAIN);

        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() &&
                !isStopRequested() &&
                (Math.abs(headingError) > HEADING_THRESHOLD) &&
                turnTimer.time() < turnTime) {

            // Determine required steering to keep on heading
            turnSpeed = getSteeringCorrection(heading, P_TURN_GAIN);

            // Clip the speed to the maximum permitted value.
            turnSpeed = Range.clip(turnSpeed, -maxTurnSpeed, maxTurnSpeed);

            // Pivot in place by applying the turning correction
            moveRobot(0, turnSpeed);
        }

        // Stop all motion;
        stopAllMotors();
//        moveRobot(0, 0);
    }

    /**
     * Method to obtain & hold a heading for a finite amount of time
     * Move will stop once the requested time has elapsed
     * This function is useful for giving the robot a moment to stabilize it's heading between movements.
     *
     * @param maxTurnSpeed Maximum differential turn speed (range 0 to +1.0)
     * @param heading      Absolute Heading Angle (in Degrees) relative to last gyro reset.
     *                     0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                     If a relative angle is required, add/subtract from current heading.
     * @param holdTime     Length of time (in seconds) to hold the specified heading.
     */
    public void holdHeading(double maxTurnSpeed, double heading, double holdTime) {

        ElapsedTime holdTimer = new ElapsedTime();
        holdTimer.reset();

        // keep looping while we have time remaining.
        while (opModeIsActive() &&
                !isStopRequested() &&
                (holdTimer.time() < holdTime)) {
            // Determine required steering to keep on heading
            turnSpeed = getSteeringCorrection(heading, P_TURN_GAIN);

            // Clip the speed to the maximum permitted value.
            turnSpeed = Range.clip(turnSpeed, -maxTurnSpeed, maxTurnSpeed);

            // Pivot in place by applying the turning correction
            moveRobot(0, turnSpeed);
        }

        // Stop all motion;
        stopAllMotors();
//        moveRobot(0, 0);
    }

    // **********  LOW Level driving functions.  ********************

    /**
     * This method uses a Proportional Controller to determine how much steering correction is required.
     *
     * @param desiredHeading   The desired absolute heading (relative to last heading reset)
     * @param proportionalGain Gain factor applied to heading error to obtain turning power.
     * @return Turning power needed to get to required heading.
     */
    public double getSteeringCorrection(double desiredHeading, double proportionalGain) {
        targetHeading = desiredHeading;  // Save for telemetry

        // Get the robot heading by applying an offset to the IMU heading
        robotHeading = getRawHeading() - headingOffset;

        // Determine the heading current error
        headingError = targetHeading - robotHeading;

        // Normalize the error to be within +/- 180 degrees
        while (headingError > 180) headingError -= 360;
        while (headingError <= -180) headingError += 360;

        // Multiply the error by the gain to determine the required steering correction/  Limit the result to +/- 1.0
        return Range.clip(headingError * proportionalGain, -1, 1);
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

        leftSpeed = drive - turn;
        rightSpeed = drive + turn;

        // Scale speeds down if either one exceeds +/- 1.0;
        double max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
        if (max > 1.0) {
            leftSpeed /= max;
            rightSpeed /= max;
        }

//        leftMotors.set(leftSpeed);
//        rightMotors.set(rightSpeed);
        leftMotors.set(simpleFeedForward.calculate(leftSpeed));
        rightMotors.set(simpleFeedForward.calculate(rightSpeed));

        sendTelemetry();
    }

    /**
     * Strafe left or right.
     *
     * @param strafeSpeed - Drive speed.
     * @param distance    - Distance in inches..
     * @param heading     - Strafe direction. Field-centric.
     * @param strafeTime  - Timeout seconds.
     */
    public void strafeRobot(double strafeSpeed, double distance, double heading, int strafeTime) {
        ElapsedTime strafeTimer = new ElapsedTime();
        strafeTimer.reset();
        backLeftDrive.resetEncoder();
        backRightDrive.resetEncoder();
        frontLeftDrive.resetEncoder();
        frontRightDrive.resetEncoder();

        targetHeading = heading;
        int moveCounts = (int) (distance * countsPerInch);
        getCurrentPositionsFromMotors();
        backLeftTarget = backLeftPosition + moveCounts;
        backRightTarget = backRightPosition + moveCounts;
        frontLeftTarget = frontLeftPosition + moveCounts;
        frontRightTarget = frontRightPosition + moveCounts;

        if (heading > 180) {
            frontLeftTarget *= -1;
            backRightTarget *= -1;
        } else {
            frontRightTarget *= -1;
            backLeftTarget *= -1;
        }

        // Set Target FIRST, then turn on RUN_TO_POSITION
        backLeftDrive.setTargetPosition(backLeftTarget);
        backRightDrive.setTargetPosition(backRightTarget);
        frontLeftDrive.setTargetPosition(frontLeftTarget);
        frontRightDrive.setTargetDistance(frontRightTarget);

        backLeftDrive.setRunMode(MotorEx.RunMode.PositionControl);
        backRightDrive.setRunMode(MotorEx.RunMode.PositionControl);
        frontLeftDrive.setRunMode(Motor.RunMode.PositionControl);
        frontRightDrive.setRunMode(Motor.RunMode.PositionControl);

        while (opModeIsActive() &&
                !isStopRequested() &&
                !(backLeftDrive.atTargetPosition() &&
                        backRightDrive.atTargetPosition() &&
                        frontLeftDrive.atTargetPosition() &&
                        frontRightDrive.atTargetPosition()) &&
                (strafeTimer.seconds() < strafeTime)) {
            backLeftDrive.set(strafeSpeed);
            backRightDrive.set(strafeSpeed);
            frontLeftDrive.set(strafeSpeed);
            frontRightDrive.set(strafeSpeed);
            sendTelemetry();
        }

        stopAllMotors();
    }

    public void moveArm(RHSArmMotorHold.ArmPosition position) {
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
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setVelocityPIDFCoefficients(1.26, 0.126, 0, 12.6);
        armMotor.setPositionPIDFCoefficients(8);
        armMotor.setVelocity(MAX_VELOCITY);

        while (armMotor.isBusy() && !isStopRequested()) {
            armVelocity = armMotor.getVelocity();
            feedForwardCalculate = armFeedForward.calculate(armVelocity);
            armMotor.setPower(feedForwardCalculate);
            armPosition = armMotor.getCurrentPosition();
            armCurrent = armMotor.getCurrent(CurrentUnit.AMPS);
            isBusy = armMotor.isBusy();
            logData();
            sendTelemetry();
        }
    }

    /**
     * Display the various control parameters while driving
     */
    private void sendTelemetry() {
        telemetry.addData("Path Segment", pathSegment);
        telemetry.addData("Target FL:FR",
                "%7d:%7d\n       BL:BR %7d:%7d",
                frontLeftTarget,
                frontRightTarget,
                backLeftTarget,
                backRightTarget);
        telemetry.addData("Actual FL:FR",
                "%7d:%7d\n       BL:BR %7d:%7d",
                frontLeftPosition,
                frontRightPosition,
                backLeftPosition,
                backRightPosition);
        telemetry.addData("Angle Target:Current", "%5.2f:%5.0f", targetHeading, robotHeading);
        telemetry.addData("Error:Steer", "%5.1f:%5.1f", headingError, turnSpeed);
        telemetry.addData("Wheel Speeds L:R.", "%5.2f : %5.2f", leftSpeed, rightSpeed);
        telemetry.update();
    }

    // This get positions from motor groups.
    public void getCurrentPositionsFromMotorGroups() {
        List<Double> leftPositions = leftMotors.getPositions();
        List<Double> rightPositions = rightMotors.getPositions();
        backLeftPosition = leftPositions.get(0).intValue();
        frontLeftPosition = leftPositions.get(1).intValue();
        backRightPosition = rightPositions.get(0).intValue();
        frontRightPosition = rightPositions.get(1).intValue();
    }

    // Get postions from individual motors.
    public void getCurrentPositionsFromMotors() {
        backLeftPosition = backLeftDrive.getCurrentPosition();
        backRightPosition = backRightDrive.getCurrentPosition();
        frontLeftPosition = frontLeftDrive.getCurrentPosition();
        frontRightPosition = frontRightDrive.getCurrentPosition();
    }

    public void stopAllMotors() {
        backLeftDrive.stopMotor();
        backRightDrive.stopMotor();
        frontLeftDrive.stopMotor();
        frontRightDrive.stopMotor();
    }

    /**
     * read the raw (un-offset Gyro heading) directly from the IMU
     */
    public double getRawHeading() {
        Orientation angles = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return angles.firstAngle;
    }

    /**
     * Reset the "offset" heading back to zero
     */
    public void resetHeading() {
        // Save a new heading offset equal to the current raw heading.
        headingOffset = getRawHeading();
        robotHeading = 0;
    }

    public void openGripper() {

        gripperServo.turnToAngle(GRIPPER_OPEN);
    }

    public void closeGripper() {
        gripperServo.turnToAngle(GRIPPER_CLOSED);
    }

    public void logData() {
        dataLog.target.set(armTarget);
        dataLog.position.set(armPosition);
        dataLog.velocity.set(armVelocity);
        dataLog.current.set(armCurrent);
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

    public enum StartPosition {
        RIGHT,
        LEFT,
        NONE
    }    /*
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