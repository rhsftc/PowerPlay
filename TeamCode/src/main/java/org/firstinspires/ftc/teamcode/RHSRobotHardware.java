package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.controller.wpilibcontroller.ElevatorFeedforward;
import com.arcrobotics.ftclib.controller.wpilibcontroller.SimpleMotorFeedforward;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;

import java.util.function.BooleanSupplier;

public class RHSRobotHardware {    /* Declare OpMode members. */
    // Calculate the COUNTS_PER_INCH for your specific drive train.
    // Go to your motor vendor website to determine your motor's COUNTS_PER_MOTOR_REV
    // For external drive gearing, set DRIVE_GEAR_REDUCTION as needed.
    // For example, use a value of 2.0 for a 12-tooth spur gear driving a 24-tooth spur gear.
    // This is gearing DOWN for less speed and more torque.
    // For gearing UP, use a gear ratio less than 1.0. Note this will affect the direction of wheel rotation.
    public final double DRIVE_GEAR_REDUCTION = 1.0;     // No External Gearing.
    public final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    public double GRIPPER_MIN_ANGLE = 0;
    public double GRIPPER_MAX_ANGLE = 360;
    public double GRIPPER_OPEN = 255;
    public double GRIPPER_CLOSED = 0;
    public IMU imu;
    public MotorEx backLeftDrive;
    public MotorEx backRightDrive;
    public MotorEx frontLeftDrive;
    public MotorEx frontRightDrive;
    public SimpleMotorFeedforward simpleFeedForward;
    public MecanumDrive driveRobot;
    public MotorEx ArmMotor;
    public ElevatorFeedforward armFeedForward;
    public GamepadEx gamePadArm;
    public GamepadEx gamePadDrive;
    public SimpleServo servo;
    public BooleanSupplier openClaw;
    public BooleanSupplier closeClaw;
    private LinearOpMode rhsOpMode = null;   // gain access to methods in the calling OpMode.

    public RHSRobotHardware(LinearOpMode opMode) {
        rhsOpMode = opMode;
    }

    public void init() {
        imu = rhsOpMode.hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        // Now initialize the IMU with this mounting orientation
        // Note: if you choose two conflicting directions, this initialization will cause a code exception.
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        ArmMotor = new MotorEx(rhsOpMode.hardwareMap, "leftbackdrive", Motor.GoBILDA.RPM_435);
        armFeedForward = new ElevatorFeedforward(10, 20, 30);

        backLeftDrive = new MotorEx(rhsOpMode.hardwareMap, "leftbackdrive");
        backRightDrive = new MotorEx(rhsOpMode.hardwareMap, "rightbackdrive");
        frontLeftDrive = new MotorEx(rhsOpMode.hardwareMap, "leftfrontdrive");
        frontRightDrive = new MotorEx(rhsOpMode.hardwareMap, "rightfrontdrive");
        simpleFeedForward = new SimpleMotorFeedforward(10, 20);

        backLeftDrive.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        frontLeftDrive.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        backLeftDrive.setInverted(true);
        frontLeftDrive.setInverted(true);

        driveRobot = new MecanumDrive(frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive);

        servo = new SimpleServo(rhsOpMode.hardwareMap, "servo1", GRIPPER_MIN_ANGLE, GRIPPER_MAX_ANGLE);
        servo.setRange(GRIPPER_CLOSED, GRIPPER_OPEN);
        gamePadDrive = new GamepadEx(rhsOpMode.gamepad1);
        gamePadArm = new GamepadEx(rhsOpMode.gamepad2);
        // Setup claw control
        openClaw = () -> gamePadArm.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)
                && !gamePadArm.isDown(GamepadKeys.Button.RIGHT_BUMPER);
        closeClaw = () -> !gamePadArm.isDown(GamepadKeys.Button.LEFT_BUMPER)
                && gamePadArm.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER);

        driveRobot = new MecanumDrive(frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive);
    }
}
