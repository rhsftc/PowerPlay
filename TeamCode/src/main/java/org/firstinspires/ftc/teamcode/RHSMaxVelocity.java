package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Max Velocity", group = "motor")
//@Disabled
public class RHSMaxVelocity extends LinearOpMode {
    DcMotorEx motor;
    double currentVelocity;
    double targetPosition = 0;
    double currentPosition = 0;
    double maxVelocity = 0.0;
    double runVelocity = 0.0;
    double pidfP = 0;
    double pidfI = 0;
    double pidfD = 0;
    double pidfF = 0;
    ElapsedTime timer;

    @Override
    public void runOpMode() {
        timer = new ElapsedTime();
        motor = hardwareMap.get(DcMotorEx.class, "leftbackdrive");
        waitForStart();
        while (opModeIsActive()) {
            timer.reset();
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motor.setPower(1);
            while (timer.seconds() < 4 && !isStopRequested()) {
                currentVelocity = motor.getVelocity();
                if (currentVelocity > maxVelocity) {
                    maxVelocity = currentVelocity;
                }
                telemetry.addData("current velocity", currentVelocity);
                telemetry.addData("maximum velocity", maxVelocity);
                telemetry.update();
            }

            motor.setPower(0);
            pidfF = 32767 / maxVelocity;
            pidfP = pidfF * .1;
            pidfI = pidfP * .1;

            sleep(2000);
            runVelocity = maxVelocity * .75;
            targetPosition = motor.getCurrentPosition() + 8000;
            motor.setTargetPosition(motor.getCurrentPosition() + 8000);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor.setVelocityPIDFCoefficients(pidfP, pidfI, pidfD, pidfF);
            motor.setPositionPIDFCoefficients(10);
            motor.setTargetPositionTolerance(5);
            motor.setVelocity(runVelocity);
            while (!isStopRequested() && motor.isBusy()) {
                currentPosition = motor.getCurrentPosition();
                currentVelocity = motor.getVelocity();
                telemetry.addData("PIDF", "P=%g I=%g D=%g F=%g", pidfP, pidfI, pidfD, pidfF);
                telemetry.addData("Run to position, velocity ", runVelocity);
                telemetry.addData("Target", targetPosition);
                telemetry.addData("current position", currentPosition);
                telemetry.addData("current velocity", currentVelocity);
                telemetry.addData("Busy", motor.isBusy());
                telemetry.update();
            }

            telemetry.addData("Position Tolerance", targetPosition - currentPosition);
            telemetry.addData("PIDF", "P=%g I=%g D=%g F=%g", pidfP, pidfI, pidfD, pidfF);
            telemetry.update();
            sleep(5000);
        }
    }
}