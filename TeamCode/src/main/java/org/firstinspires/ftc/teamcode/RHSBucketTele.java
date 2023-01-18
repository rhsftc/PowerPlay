package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.controller.wpilibcontroller.ElevatorFeedforward;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.List;
import java.util.function.BooleanSupplier;


@TeleOp(name = "Bucket Tele", group = "test")
//@Disabled
public class RHSBucketTele extends LinearOpMode {
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // No External Gearing.
    static final double WHEEL_DIAMETER_INCHES = 3.778;
    static final double ARM_DRIVE_REDUCTION = .20;
    static final double ARM_WHEEL_DIAMETER_INCHES = 2;
    static final int LOW_JUNCTION = 14;
    static final int MEDIUM_JUNCTION = 24;
    static final int HIGH_JUNCTION = 34;
    static final int HOME_POSITION = 1;
    static final int CONE_HEIGHT = 5;
    static final int ADJUST_ARM_INCREMENT = 1;
    static final double MAX_POWER = 0.4;
    // These set the range for the gripper servo.
    static final double GRIPPER_MIN_ANGLE = 0;
    static final double GRIPPER_MAX_ANGLE = 45;
    // These set the open and close positions
    static final double GRIPPER_OPEN = 12;
    static final double GRIPPER_CLOSED = 24;

    private MotorEx armMotor = null;
    private ElevatorFeedforward armFeedForward;
    private GamepadEx gamePadArm;
    private GamepadEx gamePadDrive;
    private SimpleServo GripperServo;
    private BooleanSupplier openClaw;
    private BooleanSupplier closeClaw;
    private Datalog dataLog;
    private int armTarget = 0;
    private int armPosition = 0;
    private double armVelocity = 0;
    private double armDistance = 0;
    private double armAcceleration = 0;
    // These are set in init.
    private double countsPerMotorRev = 480;
    private double motorRPM = 300;
    private double countsPerInch = 0;
    private double armCountsPerMotorRev = 0;
    private double armCountsPerInch = 0;


    public void runOpMode() {
        // Bulk reads
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        // Important: Set all Expansion hubs to use the AUTO Bulk Caching mode
        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        //TODO This arm name is temporary for testing.
        armMotor = new MotorEx(hardwareMap, "armmotor", Motor.GoBILDA.RPM_435);
        armFeedForward = new ElevatorFeedforward(10, 20, 30);

        // This is the arm motor.
        armCountsPerMotorRev = armMotor.ACHIEVABLE_MAX_TICKS_PER_SECOND;
        armCountsPerInch = ((armCountsPerMotorRev * ARM_DRIVE_REDUCTION) / (ARM_WHEEL_DIAMETER_INCHES * 3.145));

        // This is for drive motors
        countsPerInch = ((countsPerMotorRev * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415));

        MotorEx frontLeftDrive = new MotorEx(hardwareMap, "leftfrontdrive");
        MotorEx backLeftDrive = new MotorEx(hardwareMap, "leftbackdrive");
        MotorEx frontRightDrive = new MotorEx(hardwareMap, "rightfrontdrive");
        MotorEx backRightDrive = new MotorEx(hardwareMap, "rightbackdrive");

        frontLeftDrive.setInverted(true);
        backLeftDrive.setInverted(true);
        frontRightDrive.setInverted(true);
        backRightDrive.setInverted(true);

        dataLog = new Datalog("datalogarm");

        armMotor.setInverted(false);
        armMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        armMotor.setRunMode(Motor.RunMode.PositionControl);
        armMotor.setPositionCoefficient(.05);
        armMotor.setPositionTolerance(10);
        armMotor.resetEncoder();

        // Move arm to start position
        moveArm(ArmPosition.ground);

        GripperServo = new SimpleServo(hardwareMap, "servo1", GRIPPER_MIN_ANGLE, GRIPPER_MAX_ANGLE, AngleUnit.DEGREES);
        GripperServo.setInverted(true);

        openClaw = () -> gamePadArm.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)
                && !gamePadArm.isDown(GamepadKeys.Button.RIGHT_BUMPER);
        closeClaw = () -> !gamePadArm.isDown(GamepadKeys.Button.LEFT_BUMPER)
                && gamePadArm.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER);

        gamePadDrive = new GamepadEx(gamepad1);
        gamePadArm = new GamepadEx(gamepad2);
        MecanumDrive drive = new MecanumDrive(frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive);


//        telemetry.addData("Achievable Ticks", countsPerMotorRev);
//        telemetry.addData("RPM", motorRPM);
//        telemetry.addData("Counts/inch", countsPerInch);
//        telemetry.update();

        waitForStart();
        while (opModeIsActive() && !isStopRequested()) {
            gamePadDrive.readButtons();
            gamePadArm.readButtons();

            drive.driveRobotCentric(gamePadDrive.getLeftX(),
                    gamePadDrive.getLeftY(),
                    gamePadDrive.getRightX());

            ProcessArm();
            ProcessGripper();
            SendTelemetry();
        }
    }

    public void SendTelemetry() {
        telemetry.addData("Target Position", "%d", armTarget);
        telemetry.addData("Current Position", armPosition);
        telemetry.addData("Acceleration", armAcceleration);
//        telemetry.addData("Servo Position", "%6.2f - %6.2f", GripperServo.getPosition(), GripperServo.getAngle());
        telemetry.update();
    }

    public void ProcessGripper() {
        if (openClaw.getAsBoolean()) {
            GripperServo.turnToAngle(GRIPPER_OPEN);
            moveArm(ArmPosition.ground);
        }

        if (closeClaw.getAsBoolean()) {
            GripperServo.turnToAngle(GRIPPER_CLOSED);
        }
    }

    public void ProcessArm() {
        // Adjust position
        if (gamePadArm.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
            moveArm(ArmPosition.adjustDown);
        }

        if (gamePadArm.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
            moveArm(ArmPosition.adjustUp);
        }

        // Low junction
        if (gamePadArm.wasJustPressed(GamepadKeys.Button.A)) {
            moveArm(ArmPosition.low);
        }

        // medium junction
        if (gamePadArm.wasJustPressed(GamepadKeys.Button.B)) {
            moveArm(ArmPosition.medium);
        }

        // high junction
        if (gamePadArm.wasJustPressed(GamepadKeys.Button.Y)) {
            moveArm(ArmPosition.high);
        }

        // ground junction
        if (gamePadArm.wasJustPressed(GamepadKeys.Button.X)) {
            moveArm(ArmPosition.ground);
        }
    }

    public void moveArm(ArmPosition position) {
        armPosition = armMotor.getCurrentPosition();
        switch (position) {
            case ground:
                armTarget = HOME_POSITION * (int) armCountsPerInch;
                break;
            case low:
                armTarget = (LOW_JUNCTION * (int) armCountsPerInch);
                break;
            case medium:
                armTarget = (MEDIUM_JUNCTION * (int) armCountsPerInch);
                break;
            case high:
                armTarget = (HIGH_JUNCTION * (int) armCountsPerInch);
                break;
            case adjustUp:
                armTarget = armPosition + (ADJUST_ARM_INCREMENT * (int) armCountsPerInch);
                break;
            case adjustDown:
                armTarget = armPosition - (ADJUST_ARM_INCREMENT * (int) armCountsPerInch);
                break;
            default:
                armTarget = 0;
        }

        // Prevent arm moving below HOME_POSITION
        armTarget = Math.max(armTarget, HOME_POSITION);
        armMotor.setRunMode(Motor.RunMode.PositionControl);
        armMotor.setTargetPosition(armTarget);

        while (!armMotor.atTargetPosition() && !isStopRequested()) {
//            armMotor.set(MAX_POWER);
            armMotor.set(armFeedForward.calculate(MAX_POWER));
            armPosition = armMotor.getCurrentPosition();
            armDistance = armMotor.getDistance();
            armVelocity = armMotor.getVelocity();
            armMotor.encoder.getRawVelocity();
            armAcceleration = armMotor.getAcceleration();
            LogData();
            SendTelemetry();
        }

        armMotor.stopMotor();
    }

    public void LogData() {
        dataLog.target.set(armTarget);
        dataLog.velocity.set(armVelocity);
        dataLog.position.set(armPosition);
        dataLog.distance.set(armDistance);
        dataLog.acceleration.set(armAcceleration);
        dataLog.writeLine();
    }

    public enum ArmPosition {
        ground,
        low,
        medium,
        high,
        adjustUp,
        adjustDown
    }

    public static class Datalog {
        // The underlying datalogger object - it cares only about an array of loggable fields
        private final Datalogger datalogger;

        // These are all of the fields that we want in the datalog.
        // Note that order here is NOT important. The order is important in the setFields() call below
        public Datalogger.GenericField velocity = new Datalogger.GenericField("Velocity");
        public Datalogger.GenericField target = new Datalogger.GenericField("Target");
        public Datalogger.GenericField position = new Datalogger.GenericField("Position");
        public Datalogger.GenericField distance = new Datalogger.GenericField("Distance");
        public Datalogger.GenericField acceleration = new Datalogger.GenericField("Acceleration");

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
                            distance,
                            acceleration
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