package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Max Velocity", group = "test")
//@Disabled
public class RHSMaxVelocity extends LinearOpMode {
    DcMotorEx motor;
    double currentVelocity;
    double maxVelocity = 0.0;
    double runVelocoity = 0.0;
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
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setPower(1);
            while (timer.seconds() < 3 && !isStopRequested()) {
                currentVelocity = motor.getVelocity();
                if (currentVelocity > maxVelocity) {
                    maxVelocity = currentVelocity;
                }
                telemetry.addData("current velocity", currentVelocity);
                telemetry.addData("maximum velocity", maxVelocity);
                telemetry.update();
            }

            pidfF = 32767 / maxVelocity;
            pidfP = pidfF * .1;
            pidfI = pidfP * .1;


            sleep(2000);
            runVelocoity = maxVelocity * .75;
            motor.setTargetPosition(motor.getCurrentPosition() + 8000);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor.setVelocityPIDFCoefficients(pidfP, pidfI, pidfD, pidfF);
            motor.setPositionPIDFCoefficients(5);
            motor.setVelocity(runVelocoity);
            timer.reset();
            while (timer.seconds() < 5 && !isStopRequested()) {
                telemetry.addData("PIDF", "P=%g I=%g D=%g F=%g", pidfP, pidfI, pidfD, pidfF);
                telemetry.addData("Run to position, velocity ", runVelocoity);
                telemetry.addData("current position", motor.getCurrentPosition());
                telemetry.addData("current velocity", motor.getVelocity());
                telemetry.update();
            }

            sleep(5000);
        }
    }
}