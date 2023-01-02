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

@TeleOp(name = "Bucket Tele", group = "FtcLib")
//@Disabled
public class RHSBucketTele extends LinearOpMode {
    int LOW_JUNCTION = 1700;
    int MEDIUM_JUNCTION = 2800;
    int HIGH_JUNCTION = 3800;
    int HOME_POSITION = 0;
    int CONE_HEIGHT = 750;
    double MAX_POWER = 0.4;
    double GRIPPER_OPEN = 255;
    double GRIPPER_CLOSED = 0;
    double GRIPPER_RANGE = 360;

    private MotorEx ArmMotor;
    private ElevatorFeedforward armFeedForward;
    private GamepadEx gamePadArm;
    private SimpleServo GripperServo;
    private BooleanSupplier openClaw;
    private BooleanSupplier closeClaw;

    public void runOpMode() {
        ArmMotor = new MotorEx(hardwareMap, "armmotor", Motor.GoBILDA.RPM_435);
        armFeedForward = new ElevatorFeedforward(20,10,10);
        MotorEx frontLeftDrive = new MotorEx(hardwareMap, "leftfrontdrive", Motor.GoBILDA.RPM_435);
        MotorEx backLeftDrive = new MotorEx(hardwareMap, "leftbackdrive", Motor.GoBILDA.RPM_435);
        MotorEx frontRightDrive = new MotorEx(hardwareMap, "rightfrontdrive", Motor.GoBILDA.RPM_435);
        MotorEx backRightDrive = new MotorEx(hardwareMap, "rightbackbackdrive", Motor.GoBILDA.RPM_435);
        frontLeftDrive.setInverted(true);
        backLeftDrive.setInverted(true);

        ArmMotor.setInverted(true);
        ArmMotor.setTargetPosition(0);
        ArmMotor.setRunMode(Motor.RunMode.PositionControl);

        GripperServo = new SimpleServo( hardwareMap, "servo1", 0,GRIPPER_RANGE, AngleUnit.DEGREES);
        openClaw = () -> gamePadArm.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)
                && !gamePadArm.isDown(GamepadKeys.Button.RIGHT_BUMPER);
        closeClaw = () -> !gamePadArm.isDown(GamepadKeys.Button.LEFT_BUMPER)
                && gamePadArm.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER);

        // Bulk reads
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        // Important: Set all Expansion hubs to use the AUTO Bulk Caching mode
        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        GamepadEx gamePadDrive = new GamepadEx(gamepad1);
        gamePadArm = new GamepadEx(gamepad2);
        MecanumDrive drive = new MecanumDrive(frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive);

        waitForStart();
        while (opModeIsActive() && !isStopRequested()) {
            gamePadDrive.readButtons();
            gamePadArm.readButtons();

            drive.driveRobotCentric(gamePadDrive.getLeftX(),
                    gamePadDrive.getLeftY(),
                    gamePadDrive.getRightX());

            ProcessArm();
            ProcessGripper();

            telemetry.addData("Ticks", ArmMotor.getCurrentPosition());
            telemetry.addData("key", GripperServo.getPosition());
            telemetry.update();
        }
    }

    public void ProcessGripper() {
        if (openClaw.getAsBoolean()) {
            GripperServo.turnToAngle(GRIPPER_OPEN);
            sleep(100);
            ArmMotor.setTargetPosition(ArmMotor.getCurrentPosition() + CONE_HEIGHT);
            ArmMotor.set(MAX_POWER);
        }

        if (closeClaw.getAsBoolean()) {
            GripperServo.turnToAngle(GRIPPER_CLOSED);
        }
    }

    public void ProcessArm() {
        // Wait for arm to be idle.
        if (!ArmMotor.atTargetPosition() && ArmMotor.motorEx.isBusy()) {
            return;
        }

        // Adjust position
        if (gamePadArm.isDown(GamepadKeys.Button.DPAD_DOWN)) {
            ArmMotor.setTargetPosition(ArmMotor.getCurrentPosition() - 100);
            ArmMotor.set(armFeedForward.calculate(MAX_POWER));
        }

        if (gamePadArm.isDown(GamepadKeys.Button.DPAD_UP)) {
            ArmMotor.setTargetPosition(ArmMotor.getCurrentPosition() + 100);
            ArmMotor.set(armFeedForward.calculate(MAX_POWER));
        }

        // Low junction
        if (gamePadArm.isDown(GamepadKeys.Button.A)) {
            if (ArmMotor.getCurrentPosition() > 3 && GripperServo.getPosition() == 0) {
                ArmMotor.setTargetPosition(ArmMotor.getCurrentPosition() + (LOW_JUNCTION - ArmMotor.getCurrentPosition()));
                ArmMotor.setRunMode(Motor.RunMode.PositionControl);
                ArmMotor.set(armFeedForward.calculate(MAX_POWER));
            } else {
                ArmMotor.setTargetPosition(HOME_POSITION);
                ArmMotor.setRunMode(Motor.RunMode.PositionControl);
                ArmMotor.set(armFeedForward.calculate(MAX_POWER));
            }
        }

        if (gamePadArm.isDown(GamepadKeys.Button.B)) {
            if (ArmMotor.getCurrentPosition() > 3 && GripperServo.getPosition() == 0) {
                ArmMotor.setTargetPosition(ArmMotor.getCurrentPosition() + (MEDIUM_JUNCTION - ArmMotor.getCurrentPosition()));
                ArmMotor.setRunMode(Motor.RunMode.PositionControl);
                ArmMotor.set(armFeedForward.calculate(MAX_POWER));
            } else {
                ArmMotor.setTargetPosition(HOME_POSITION);
                ArmMotor.setRunMode(Motor.RunMode.PositionControl);
                ArmMotor.set(armFeedForward.calculate(MAX_POWER));
            }
        }

        if (gamePadArm.isDown(GamepadKeys.Button.Y)) {
            if (ArmMotor.getCurrentPosition() > 3 && GripperServo.getPosition() == 0) {
                ArmMotor.setTargetPosition(ArmMotor.getCurrentPosition() + (HIGH_JUNCTION - ArmMotor.getCurrentPosition()));
                ArmMotor.setRunMode(Motor.RunMode.PositionControl);
                ArmMotor.set(armFeedForward.calculate(MAX_POWER));
            } else {
                ArmMotor.setTargetPosition(HOME_POSITION);
                ArmMotor.setRunMode(Motor.RunMode.PositionControl);
                ArmMotor.set(armFeedForward.calculate(MAX_POWER));
            }
        }

        if (gamePadArm.isDown(GamepadKeys.Button.X)) {
            ArmMotor.setTargetPosition(HOME_POSITION);
            ArmMotor.setRunMode(Motor.RunMode.PositionControl);
            ArmMotor.set(armFeedForward.calculate(MAX_POWER));
        }
    }
}