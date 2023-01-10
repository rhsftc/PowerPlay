/*
This sample FTC OpMode uses methods of the Datalogger class to specify and
collect robot data to be logged in a CSV file, ready for download and charting.

For instructions, see the tutorial at the FTC Wiki:
https://github.com/FIRST-Tech-Challenge/FtcRobotController/wiki/Datalogging


The Datalogger class is suitable for FTC OnBot Java (OBJ) programmers.
Its methods can be made available for FTC Blocks, by creating myBlocks in OBJ.

Android Studio programmers can see instructions in the Datalogger class notes.

Credit to @Windwoes (https://github.com/Windwoes).

*/


package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@TeleOp(name = "IMU Datalogger", group = "test")
public class RHSImuDatalogger extends LinearOpMode {
    Datalog datalog;
    IMU imu;
    VoltageSensor battery;

    @Override
    public void runOpMode() throws InterruptedException {
        // Get devices from the hardwareMap.
        // If needed, change "Control Hub" to (e.g.) "Expansion Hub 1".
        battery = hardwareMap.voltageSensor.get("Control Hub");
        imu = hardwareMap.get(IMU.class, "imu");

        // Initialize the datalog
        // Note that the order in which we set datalog fields
        // does *not* matter! The order is configured inside
        // the Datalog class constructor.
        datalog = new Datalog("datalogImu");

        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        // Now initialize the IMU with this mounting orientation
        // Note: if you choose two conflicting directions, this initialization will cause a code exception.
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        imu.resetYaw();

        telemetry.setMsTransmissionInterval(50);

        waitForStart();

        for (int i = 0; opModeIsActive(); i++) {

            YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
            AngularVelocity angularVelocity = imu.getRobotAngularVelocity(AngleUnit.DEGREES);

            datalog.yaw.set(orientation.getYaw(AngleUnit.DEGREES));
            datalog.pitch.set(orientation.getPitch(AngleUnit.DEGREES));
            datalog.roll.set(orientation.getRoll(AngleUnit.DEGREES));
            datalog.zRate.set(angularVelocity.zRotationRate);
            datalog.xRate.set(angularVelocity.xRotationRate);
            datalog.yRate.set(angularVelocity.yRotationRate);

            // The logged timestamp is taken when writeLine() is called.
            datalog.writeLine();

            // Datalog fields are stored as text only; do not format here.
            telemetry.addData("Yaw", datalog.yaw);
            telemetry.addData("Pitch", datalog.pitch);
            telemetry.addData("Roll", datalog.roll);
            telemetry.addLine();
            telemetry.addData("Z Rate", angularVelocity.zRotationRate);
            telemetry.addData("X Rate", angularVelocity.xRotationRate);
            telemetry.addData("Y Rate", angularVelocity.yRotationRate);
            telemetry.update();

            sleep(20);
        }

        /*
         * The datalog is automatically closed and flushed to disk after
         * the OpMode ends - no need to do that manually :')
         */
    }

    /*
     * This class encapsulates all the fields that will go into the datalog.
     */
    public static class Datalog {
        // The underlying datalogger object - it cares only about an array of loggable fields
        private final Datalogger datalogger;

        // These are all of the fields that we want in the datalog.
        // Note that order here is NOT important. The order is important in the setFields() call below
        public Datalogger.GenericField yaw = new Datalogger.GenericField("Yaw");
        public Datalogger.GenericField pitch = new Datalogger.GenericField("Pitch");
        public Datalogger.GenericField roll = new Datalogger.GenericField("Roll");
        public Datalogger.GenericField zRate = new Datalogger.GenericField("Z Rate");
        public Datalogger.GenericField xRate = new Datalogger.GenericField("X Rate");
        public Datalogger.GenericField yRate = new Datalogger.GenericField("Y Rate");

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
                            yaw,
                            pitch,
                            roll,
                            zRate,
                            xRate,
                            yRate
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
