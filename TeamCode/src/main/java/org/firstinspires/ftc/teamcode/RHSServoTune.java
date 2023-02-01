package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp(name = "Servo Tune", group = "test")
//@Disabled
public class RHSServoTune extends OpMode {
    SimpleServo servo;
    boolean isInverted = false;
    private final double MIN_RANGE = 0;
    private final double MAX_RANGE = 180;
    private double turnToMinAngle = 25;
    private double turnToMaxAngle = 73;
    private GamepadEx gamePadDrive;
    private Telemetry.Item teleInverted;
    private Telemetry.Item teleTurnToMinAngle;
    private Telemetry.Item teleTurnToMaxAngle;
    private Telemetry.Item teleServoAngle;
    private Telemetry.Item teleServoPosition;
    private Telemetry.Item teleServoRange;

    @Override
    public void init() {
        gamePadDrive = new GamepadEx(gamepad1);
        servo = new SimpleServo(hardwareMap, "servo1", MIN_RANGE, MAX_RANGE);
        servo.setInverted(isInverted);

        telemetry.setAutoClear(false);
        telemetry.addLine("Back: Toggle Inverted");
        telemetry.addLine("DPAD Left: Decrease min angle");
        telemetry.addLine("DPAD Right: Increase min angle");
        telemetry.addLine("DPAD Up: Increase max angle");
        telemetry.addLine("DPAD Down: Decrease max angle");
        telemetry.addLine("A: Turn to min angle");
        telemetry.addLine("B: Turn to max angle");

        teleInverted = telemetry.addData("Inverted", isInverted);
        teleTurnToMinAngle = telemetry.addData("Min Angle", turnToMinAngle);
        teleTurnToMaxAngle = telemetry.addData("Max Angle", turnToMaxAngle);
        teleServoAngle = telemetry.addData("Servo angle", servo.getAngle());
        teleServoPosition = telemetry.addData("Servo position", servo.getPosition());
        teleServoRange = telemetry.addData("Servo range", servo.getAngleRange());
    }

    @Override
    public void init_loop() {
    }

    /*
     * This method will be called ONCE when start is pressed
     */
    @Override
    public void start() {

    }

    /*
     * This method will be called repeatedly in a loop
     */
    @Override
    public void loop() {
        gamePadDrive.readButtons();
        if (gamePadDrive.wasJustPressed(GamepadKeys.Button.BACK)) {
            isInverted = !isInverted;
        }

        if (gamePadDrive.wasJustReleased(GamepadKeys.Button.DPAD_LEFT)) {
            if (turnToMinAngle > MIN_RANGE) {
                turnToMinAngle -= 1;
            }
        }

        if (gamePadDrive.wasJustReleased(GamepadKeys.Button.DPAD_RIGHT)) {
            if (turnToMinAngle < MAX_RANGE) {
                turnToMinAngle += 1;
            }
        }

        if (gamePadDrive.wasJustReleased(GamepadKeys.Button.DPAD_UP)) {
            if (turnToMaxAngle < MAX_RANGE) {
                turnToMaxAngle += 1;
            }
        }

        if (gamePadDrive.wasJustReleased(GamepadKeys.Button.DPAD_DOWN)) {
            if (turnToMaxAngle > MIN_RANGE) {
                turnToMaxAngle -= 1;
            }
        }

        if (gamePadDrive.wasJustReleased(GamepadKeys.Button.A)) {
            servo.turnToAngle(turnToMinAngle);
        }

        if (gamePadDrive.wasJustReleased(GamepadKeys.Button.B)) {
            servo.turnToAngle(turnToMaxAngle);
        }

        sendTelemetry();
    }

    @Override
    public void stop() {

    }

    public void sendTelemetry() {
        teleInverted.setValue(isInverted);
        teleTurnToMinAngle.setValue(turnToMinAngle);
        teleTurnToMaxAngle.setValue(turnToMaxAngle);
        teleServoAngle.setValue(servo.getAngle());
        teleServoPosition.setValue(servo.getPosition());
        teleServoRange.setValue(servo.getAngleRange());
    }
}
