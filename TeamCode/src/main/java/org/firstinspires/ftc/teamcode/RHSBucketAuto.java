package org.firstinspires.ftc.teamcode;


import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.List;

@Autonomous(name = "Bucket Auto", group = "FtcLib")
@Disabled
public class RHSBucketAuto extends LinearOpMode {
    int blpos;
    int brpos;
    int flpos;
    int frpos;
    int Step_;
    private SleeveDetection sleeveDetection;
    private OpenCvCamera camera;
    private String WE_CAM_NAME = "webcam1";
    private SleeveDetection.ParkingPosition parkLocation;
    private DcMotor BackleftAsDcMotor;
    private DcMotor Backright;
    private DcMotor FrontleftAsDcMotor;
    private DcMotor Frontright;
    private SimpleServo GripperServo;

    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {
        // Bulk reads
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        // Important: Set all Expansion hubs to use the AUTO Bulk Caching mode
        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, WE_CAM_NAME), cameraMonitorViewId);
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

        while (!isStarted()) {
            telemetry.addData("ROTATION: ", sleeveDetection.getPosition());
            telemetry.update();
        }

        double Inch;
        double Tile_length;
        double Shift;
        double speed_value;

        BackleftAsDcMotor = hardwareMap.get(DcMotor.class, "Backleft");
        Backright = hardwareMap.get(DcMotor.class, "Backright");
        FrontleftAsDcMotor = hardwareMap.get(DcMotor.class, "Frontleft");
        Frontright = hardwareMap.get(DcMotor.class, "Frontright");
        GripperServo = new SimpleServo(hardwareMap, "GripperServo", 0, 255, AngleUnit.DEGREES);

        // Put initialization blocks here.
        GripperServo.setPosition(1);
        BackleftAsDcMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Backright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FrontleftAsDcMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Frontright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackleftAsDcMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        FrontleftAsDcMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        Inch = 76.9230769231;
        Tile_length = Inch * 23;
        blpos = 0;
        brpos = 0;
        flpos = 0;
        frpos = 0;
        Step_ = 1;
        Shift = Tile_length + Inch * 5;
        speed_value = .25;
        waitForStart();
        // move away from wall to the left
        while (opModeIsActive()) {
            switch (Step_) {
                case 1:
                    parkLocation = getParkLocation();
                    Step_ = 2;
                    break;
                case 2:
                    drive(Tile_length + Inch * 5, Tile_length + Inch * 5, Tile_length + Inch * 5, Tile_length + Inch * 5, speed_value);
                    Step_ = 3;
                    break;
                case 3:
                    if (parkLocation == SleeveDetection.ParkingPosition.LEFT) {
                        drive(Shift * 1, Shift * -1, Shift * -1, Shift * 1, 0.5);
                        drive(Inch * 3, Inch * 3, Inch * 3, Inch * 3, 1);
                        requestOpModeStop();
                    } else if (parkLocation == SleeveDetection.ParkingPosition.CENTER) {
                        drive(Inch * 3, Inch * 3, Inch * 3, Inch * 3, 1);
                        requestOpModeStop();
                    } else {
                        if (parkLocation == SleeveDetection.ParkingPosition.RIGHT) {
                            drive(Inch * -1, Inch * -1, Inch * -1, Inch * -1, 1);
                            drive(Shift * -1, Shift * 1, Shift * 1, Shift * -1, 0.2);
                            drive(Inch * 4, Inch * 4, Inch * 4, Inch * 4, 1);
                            requestOpModeStop();
                        } else {
                            requestOpModeStop();
                        }
                    }
                    Step_ = 4;
                    break;
                case 4:
                    telemetry.addData("Status", "Path complete.");
                    telemetry.update();
                    break;
                default:
                    throw new IllegalStateException("Unexpected value: " + Step_);
            }
        }
    }

    /**
     * Describe this function...
     */
    private void drive(double blTarget, double brTarget, double flTarget, double frTarget, double speed) {
        blpos += blTarget;
        brpos += brTarget;
        flpos += flTarget;
        frpos += frTarget;
        BackleftAsDcMotor.setTargetPosition(blpos);
        Backright.setTargetPosition(brpos);
        FrontleftAsDcMotor.setTargetPosition(flpos);
        Frontright.setTargetPosition(frpos);
        BackleftAsDcMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Backright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FrontleftAsDcMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Frontright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BackleftAsDcMotor.setPower(speed);
        Backright.setPower(speed);
        FrontleftAsDcMotor.setPower(speed);
        Frontright.setPower(speed);
        while (opModeIsActive() && BackleftAsDcMotor.isBusy() && Backright.isBusy() && FrontleftAsDcMotor.isBusy() && Frontright.isBusy()) {
            idle();
        }
    }

    /**
     * Describe this function...
     */
    private SleeveDetection.ParkingPosition getParkLocation() {
        return sleeveDetection.getPosition();
    }
}