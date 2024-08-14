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

@Autonomous(name = "RedLeft", group = "00-Autonomous", preselectTeleOp = "StarTech")
public class AutonomusRedLeft extends LinearOpMode {
    public static String TEAM_NAME = "StarTech";
    public static int TEAM_NUMBER = 18338;

    HardwareBox robot = new HardwareBox();

    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera

    private static final String TFOD_MODEL_FILE = "/sdcard/FIRST/tflitemodels/model_20240106_160015.tflite";

    /**
     * If we use default object, Pixels, change the labels name from "stratech" to "Pixel"
     * */
    private static final String[] LABELS = {
            "StarTech"
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
        robot.slider.setTargetPosition(0);
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
        visionPortal.close();
    }   // end runOpMode()

    public void runAutonoumousMode() {
        //Initialize Pose2d as desired
        Pose2d initPose = new Pose2d(0, 0, 0); // Starting Pose
        Pose2d moveBeyondTrussPose = new Pose2d(0,0,0);
        Pose2d dropPurplePixelPose = new Pose2d(0, 0, 0);
        Pose2d midwayPose1 = new Pose2d(0,0,0);
        Pose2d midwayPose1a = new Pose2d(0,0,0);
        Pose2d midwayPose1b = new Pose2d(0,0,0);
        Pose2d intakeStack = new Pose2d(0,0,0);
        Pose2d midwayPose2 = new Pose2d(0,0,0);
        Pose2d midwayPose2a = new Pose2d(0,0,0);
        Pose2d dropYellowPixelPose = new Pose2d(0, 0, 0);
        Pose2d parkPose = new Pose2d(0,0, 0);
        double waitSecondsBeforeDrop = 0;
        MecanumDrive drive = new MecanumDrive(hardwareMap, initPose);

        initPose = new Pose2d(0, 0, Math.toRadians(0)); //Starting pose
        moveBeyondTrussPose = new Pose2d(15,0,0);

        drive = new MecanumDrive(hardwareMap, initPose);
        switch(identifiedSpikeMarkLocation){
            case LEFT:
                dropPurplePixelPose = new Pose2d(11, 14.8, Math.toRadians(0));
                midwayPose1 = new Pose2d(5, -14.5, Math.toRadians(0));
                intakeStack = new Pose2d(42, -12.5,Math.toRadians(0));
                midwayPose1b = new Pose2d(43, -16, Math.toRadians(58));
                midwayPose2 = new Pose2d(33, -68, Math.toRadians(88));
                midwayPose2a = new Pose2d(-15, -68, Math.toRadians(88));
                dropYellowPixelPose = new Pose2d(-16, -80.6, Math.toRadians(88));
                break;
            case MIDDLE:
                dropPurplePixelPose = new Pose2d(20, -2, Math.toRadians(0));
                midwayPose1 = new Pose2d(5, 20, Math.toRadians(0));
                intakeStack = new Pose2d(33, 18,Math.toRadians(0));
                midwayPose1b = new Pose2d(39, 6, Math.toRadians(55));
                midwayPose2 = new Pose2d(29, -63.5, Math.toRadians(88));
                midwayPose2a = new Pose2d(-40, -63.5, Math.toRadians(88));
                dropYellowPixelPose = new Pose2d(-37, -73.2, Math.toRadians(88));
                break;
            case RIGHT:
                dropPurplePixelPose = new Pose2d(20, -8, Math.toRadians(-40));
                midwayPose1 = new Pose2d(7, -1, Math.toRadians(-20));
                intakeStack = new Pose2d(42, -1,Math.toRadians(0));
                midwayPose1b = new Pose2d(44, -1, Math.toRadians(60));
                midwayPose2 = new Pose2d(47, -63.5, Math.toRadians(85));
                midwayPose2a = new Pose2d(-33, -63.5, Math.toRadians(88));
                dropYellowPixelPose = new Pose2d(-31, -79.3, Math.toRadians(88));
                break;
        }

        midwayPose1a = new Pose2d(5, 25, Math.toRadians(0));


        waitSecondsBeforeDrop = 1; //TODO: Adjust time to wait for alliance partner to move from board
        parkPose = new Pose2d(60, -84, Math.toRadians(90));

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

        //TODO merge in mijloc
       /* Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(midwayPose1a.position, midwayPose1a.heading)
                        .build());*/

        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(intakeStack.position, intakeStack.heading)
                        .build());
        robot.safeWaitSeconds(0.5);
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(midwayPose1b.position, midwayPose1b.heading)
                        .build());


        robot.safeWaitSeconds(0.5);


        //TODO Move robot to midwayPose2 and to dropYellowPixelPose
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(midwayPose2.position, midwayPose2.heading)
                        .build());

        robot.safeWaitSeconds(0.5);
        /*robot.sliderUp();

        robot.safeWaitSeconds(1);*/

        //TODO Move robot to midwayPose2 and to dropYellowPixelPose
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(midwayPose2a.position, midwayPose2a.heading)
                        .strafeToLinearHeading(dropYellowPixelPose.position, dropYellowPixelPose.heading)
                        .build());


        //TODO : Code to drop Pixel on Backdrop
        robot.safeWaitSeconds(1);
        robot.sliderUp();

        robot.safeWaitSeconds(0.5);
        robot.dropPixel();
        robot.safeWaitSeconds(0.5);
        //TODO : Code to drop Pixel on Backdrop


        //TODO Move robot to park in Backstage
        /*Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(parkPose.position, parkPose.heading)
                        //.splineToLinearHeading(parkPose,0)
                        .build());*/
    }

    //method to wait safely with stop button working if needed. Use this instead of sleep


    /**
     * Initialize the TensorFlow Object Detection processor.
     */
    private void initTfod() {

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
                //.setIsModelQuantized(true)
                .setModelInputSize(300)
                .setModelAspectRatio(16.0 / 9.0)
                .build();

        VisionPortal.Builder builder = new VisionPortal.Builder();
        // Create the vision portal the easy way.
        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }
        // Choose a camera resolution. Not all cameras support all resolutions.
        //builder.setCameraResolution(new Size(640, 480));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        //builder.enableLiveView(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        //builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        //builder.setAutoStopLiveView(false);

        // Set and enable the processor.
        builder.addProcessor(tfod);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Set confidence threshold for TFOD recognitions, at any time.
        //tfod.setMinResultConfidence(0.75f);

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

            if (recognition.getLabel() == "StarTech" && recognition.getConfidence()>0.80) {
                if (x < 200) {
                    identifiedSpikeMarkLocation = IDENTIFIED_SPIKE_MARK_LOCATION.LEFT;
                } else {
                    identifiedSpikeMarkLocation = IDENTIFIED_SPIKE_MARK_LOCATION.MIDDLE;
                }
            }

        }   // end for() loop

    }   // end method runTfodTensorFlow()

}
