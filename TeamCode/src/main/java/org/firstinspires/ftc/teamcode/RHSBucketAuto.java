package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name = "Java Autonomous Camera", group = "FtcLib")
public class RHSBucketAuto extends LinearOpMode {

    SleeveDetection sleeveDetection;
    OpenCvCamera camera;
    String webcamName = "webcam1";
    int blpos;
    int brpos;
    int Permanent_color;
    String Final_Color;
    int flpos;
    int Final_Hue;
    int frpos;
    int Step_;
    private SleeveDetection.ParkingPosition parkLocation;
    private DcMotor BackleftAsDcMotor;
    private DcMotor Backright;
    private DcMotor FrontleftAsDcMotor;
    private DcMotor Frontright;
    private Servo GripperServo;

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
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, webcamName), cameraMonitorViewId);
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
        double Foot;
        double Tile_length;
        double my_1_2_Tile_Length;
        double Shift;
        double speed_value;

        BackleftAsDcMotor = hardwareMap.get(DcMotor.class, "Backleft");
        Backright = hardwareMap.get(DcMotor.class, "Backright");
        FrontleftAsDcMotor = hardwareMap.get(DcMotor.class, "Frontleft");
        Frontright = hardwareMap.get(DcMotor.class, "Frontright");
        GripperServo = hardwareMap.get(Servo.class, "GripperServo");

        // Put initialization blocks here.
        Permanent_color = 0;
        Final_Hue = 0;
        GripperServo.setPosition(0.71);
        BackleftAsDcMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Backright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FrontleftAsDcMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Frontright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackleftAsDcMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        FrontleftAsDcMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        Inch = 76.9230769231;
        Foot = Inch * 12;
        Tile_length = Inch * 23;
        my_1_2_Tile_Length = Tile_length / 2;
        blpos = 0;
        brpos = 0;
        flpos = 0;
        frpos = 0;
        Step_ = 1;
        Final_Color = "Blank";
        Shift = Tile_length + Inch * 5;
        speed_value = .25;
        waitForStart();
        // move away from wall to the left
        while (opModeIsActive()) {
            while (Step_ == 1) {
                // go forward 1 tile
                GripperServo.setPosition(0.71);
                parkLocation = getParkLocation();
                Step_ = 2;
            }
            while (Step_ == 2) {
                drive(Tile_length + Inch * 5, Tile_length + Inch * 5, Tile_length + Inch * 5, Tile_length + Inch * 5, speed_value);
                Step_ = 3;
            }
            while (Step_ == 3) {
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
                telemetry.update();
            }
        }
    }

    /**
     * Describe this function...
     */
    private SleeveDetection.ParkingPosition getParkLocation() {
        return sleeveDetection.getPosition();
    }
}