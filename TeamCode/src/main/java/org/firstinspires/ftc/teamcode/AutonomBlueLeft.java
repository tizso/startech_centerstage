package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.util.ElapsedTime.Resolution.SECONDS;

import android.util.Size;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;

@Autonomous(name = "BuleLeft", group = "00-Autonomous", preselectTeleOp = "StarTech")
public class AutonomBlueLeft extends LinearOpMode {
    public static String TEAM_NAME = "StarTech";
    public static int TEAM_NUMBER = 18338;

    HardwareBox robot = new HardwareBox();

    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera

    /**
     * The variable to store our instance of the AprilTag processor.
     */
    private AprilTagProcessor aprilTag;

    private static final String TFOD_MODEL_FILE = "/sdcard/FIRST/tflitemodels/startech2.tflite";

    /**
     * If we use default object, Pixels, change the labels name from "stratech" to "Pixel"
     * */
    private static final String[] LABELS = {
            "StarTechBLue",
            "StarTechRed"
    };

    //Vision parameters
    private TfodProcessor tfod;
    private VisionPortal visionPortal;

    public enum IDENTIFIED_SPIKE_MARK_LOCATION {
        LEFT,
        MIDDLE,
        RIGHT
    }

    public static IDENTIFIED_SPIKE_MARK_LOCATION identifiedSpikeMarkLocation = IDENTIFIED_SPIKE_MARK_LOCATION.LEFT;

    @Override
    public void runOpMode() throws InterruptedException {

        //Activate Camera Vision that uses TensorFlow for pixel detection
        initTfod();
        robot.init(hardwareMap);
        robot.slider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //colector
        robot.colector.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.slider.setDirection(DcMotorEx.Direction.FORWARD);
        robot.slider.setTargetPosition(20);
        robot.slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.slider.setPower(0.9);
        sleep(200);

        // Wait for the DS start button to be touched.
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();
        //waitForStart();

        while (!isStopRequested() && !opModeIsActive()) {

            //Run Vuforia Tensor Flow and keep watching for the identifier in the Signal Cone.
            runTfodTensorFlow();
            telemetry.addData("Vision identified Parking Location", identifiedSpikeMarkLocation);
            telemetry.update();
        }

        //Game Play Button  is pressed
        if (opModeIsActive() && !isStopRequested()) {
            //Build parking trajectory based on last detected target by vision
            runAutonoumousMode();
        }
    }   // end runOpMode()

    public void runAutonoumousMode() {
        //Initialize Pose2d as desired
        Pose2d initPose = new Pose2d(0, 0, 0); // Starting Pose
        Pose2d moveBeyondTrussPose = new Pose2d(0,0,0);
        Pose2d dropPurplePixelPose = new Pose2d(0, 0, 0);
        Pose2d midwayPose1 = new Pose2d(0,0,0);
        Pose2d dropYellowPixelPose = new Pose2d(0, 0, 0);
        Pose2d parkPose1 = new Pose2d(0,0, 0);
        Pose2d parkPose2 = new Pose2d(0,0, 0);
        double waitSecondsBeforeDrop = 0;
        MecanumDrive drive = new MecanumDrive(hardwareMap, initPose);

        initPose = new Pose2d(0, 0, Math.toRadians(0)); //Starting pose
        moveBeyondTrussPose = new Pose2d(15,0,0);

        //identified Spike Mark Location
        drive = new MecanumDrive(hardwareMap, initPose);
        switch(identifiedSpikeMarkLocation){
            case LEFT:
                dropPurplePixelPose = new Pose2d(17, 11, Math.toRadians(0));
                dropYellowPixelPose = new Pose2d(51, 36, Math.toRadians(-83));
                break;
            case MIDDLE:
                dropPurplePixelPose = new Pose2d(27, 2, Math.toRadians(0));
                dropYellowPixelPose = new Pose2d(45, 36.5,  Math.toRadians(-81));
                break;
            case RIGHT:
                dropPurplePixelPose = new Pose2d(24, -4, Math.toRadians(-45));
                dropYellowPixelPose = new Pose2d(29, 35, Math.toRadians(-74));
                break;
        }
        midwayPose1 = new Pose2d(10, 13, Math.toRadians(-45));
        waitSecondsBeforeDrop = 2; //TODO: Adjust time to wait for alliance partner to move from board

        //parking left side
        parkPose1 = new Pose2d(0, 30, Math.toRadians(-90));
        parkPose2 = new Pose2d(0, 40, Math.toRadians(-90));

        //parking right side
        /*parkPose1 = new Pose2d(60, 30, Math.toRadians(-90));
        parkPose2 = new Pose2d(60, 35, Math.toRadians(-90));*/


        //Move robot to dropPurplePixel based on identified Spike Mark Location
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(moveBeyondTrussPose.position, moveBeyondTrussPose.heading)
                        .strafeToLinearHeading(dropPurplePixelPose.position, dropPurplePixelPose.heading)
                        .build());

        //TODO : Code to drop Purple Pixel on Spike Mark

        robot.putPurplePixel();

        //TODO : Code to drop Purple Pixel on Spike Mark


        //Move robot to midwayPose1
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(midwayPose1.position, midwayPose1.heading)
                        .build());

        robot.safeWaitSeconds(waitSecondsBeforeDrop);

        //Move robot to midwayPose2 and to dropYellowPixelPose
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .setReversed(true)
                        .splineToLinearHeading(dropYellowPixelPose,0)
                        .build());
        //TODO : Code to drop Pixel on Backdrop

        robot.dropPixel();

        //TODO : Code to drop Pixel on Backdrop
        robot.safeWaitSeconds(1);

        //Move robot to park in Backstage
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(parkPose1.position, parkPose1.heading)
                        //.splineToLinearHeading(parkPose,0)
                        .build());
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(parkPose2.position, parkPose2.heading)
                        //.splineToLinearHeading(parkPose,0)
                        .build());
    }


    //method to wait safely with stop button working if needed. Use this instead of sleep
    /*public void safeWaitSeconds(double time) {
        ElapsedTime timer = new ElapsedTime(SECONDS);
        timer.reset();
        while (!isStopRequested() && timer.time() < time) {
        }
    }*/

    /**
     * Initialize the TensorFlow Object Detection processor.
     */
    private void initTfod() {
        // -----------------------------------------------------------------------------------------
        // AprilTag Configuration
        // -----------------------------------------------------------------------------------------
        double fx = 946.461;
        double fy = 946.136;
        double cx = 312.211;
        double cy = 211.465;
        aprilTag = new AprilTagProcessor.Builder()
                .setLensIntrinsics(fx, fy, cx, cy)
                .build();

        tfod = new TfodProcessor.Builder()
                // With the following lines commented out, the default TfodProcessor Builder
                // will load the default model for the season. To define a custom model to load,
                // choose one of the following:
                //   Use setModelAssetName() if the custom TF Model is built in as an asset (AS only).
                //   Use setModelFileName() if you have downloaded a custom team model to the Robot Controller.
                //.setModelAssetName(TFOD_MODEL_ASSET)
                .setModelFileName(TFOD_MODEL_FILE)

                // The following default settings are available to un-comment and edit as needed to
                // set parameters for custom models.
                .setModelLabels(LABELS)
                .setIsModelTensorFlow2(true)
                .setIsModelQuantized(true)
                .setModelInputSize(1200)
                .setModelAspectRatio(16.0 / 9.0)
                .build();
        tfod.setMinResultConfidence(0.75f);

        // Create the vision portal the easy way.
        if (USE_WEBCAM) {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                    .addProcessors(tfod, aprilTag)
                    .enableLiveView(true)
                    .setCameraResolution(new Size(640, 480))
                    .build();
        } else {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(BuiltinCameraDirection.BACK)
                    .addProcessors(tfod, aprilTag)
                    .build();
        }


    }   // end method initTfod()

    /**
     * Add telemetry about TensorFlow Object Detection (TFOD) recognitions.
     */
    private void runTfodTensorFlow() {

        List<Recognition> currentRecognitions = tfod.getRecognitions();
        telemetry.addData("# Objects Detected", currentRecognitions.size());

        //Camera placed between Left and Right Spike Mark on RED_LEFT and BLUE_LEFT If pixel not visible, assume Right spike Mark
        identifiedSpikeMarkLocation = IDENTIFIED_SPIKE_MARK_LOCATION.RIGHT;

        // Step through the list of recognitions and display info for each one.
        for (Recognition recognition : currentRecognitions) {
            double x = (recognition.getLeft() + recognition.getRight()) / 2 ;
            double y = (recognition.getTop()  + recognition.getBottom()) / 2 ;

            telemetry.addData(""," ");
            telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
            telemetry.addData("- Position", "%.0f / %.0f", x, y);
            telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());

            if (recognition.getLabel() == "StarTechBLue" && recognition.getConfidence()>0.75) {
                if (x < 200) {
                    identifiedSpikeMarkLocation = IDENTIFIED_SPIKE_MARK_LOCATION.LEFT;
                } else {
                    identifiedSpikeMarkLocation = IDENTIFIED_SPIKE_MARK_LOCATION.MIDDLE;
                }
            }

        }   // end for() loop

    }   // end method runTfodTensorFlow()
}


