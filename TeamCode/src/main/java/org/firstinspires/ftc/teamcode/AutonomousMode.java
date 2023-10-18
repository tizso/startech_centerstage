/* Copyright (c) 2019 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;

/**
 * FTC WIRES Autonomous Example for only vision detection using tensorflow and park
 */
@Autonomous(name = "FTC Wires Autonomous Mode", group = "00-Autonomous", preselectTeleOp = "FTC Wires TeleOp")
public class AutonomousMode extends LinearOpMode {

    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera

    //Vision parameters
    private TfodProcessor tfod;
    private VisionPortal visionPortal;

    //Define and declare Robot Starting Locations
    public enum START_POSITION{
        BLUE_LEFT,
        BLUE_RIGHT,
        RED_LEFT,
        RED_RIGHT
    }
    public static START_POSITION startPosition;

    public enum IDENTIFIED_SPIKE_MARK_LOCATION {
        LEFT,
        MIDDLE,
        RIGHT
    }
    public static IDENTIFIED_SPIKE_MARK_LOCATION identifiedSpikeMarkLocation = IDENTIFIED_SPIKE_MARK_LOCATION.LEFT;

    @Override
    public void runOpMode() throws InterruptedException {

        //Key Pay inputs to selecting Starting Position of robot
        selectStartingPosition();
        telemetry.addData("Selected Starting Position", startPosition);

        //Activate Camera Vision that uses TensorFlow for pixel detection
        initTfod();

        // Wait for the DS start button to be touched.
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();
        waitForStart();

        while (!isStopRequested() && !opModeIsActive()) {
            telemetry.addData("Selected Starting Position", startPosition);

            //Run Vuforia Tensor Flow and keep watching for the identifier in the Signal Cone.
            runTfodTensorFlow();
            telemetry.addData("Vision identified Parking Location", identifiedSpikeMarkLocation);
            telemetry.update();
        }

        //Stop Vision process
        visionPortal.close();

        //Game Play Button  is pressed
        if (opModeIsActive() && !isStopRequested()) {
            //Build parking trajectory based on last detected target by vision
            runAutonoumousMode();
        }
    }   // end runOpMode()

    public void runAutonoumousMode() {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        //Initialize any other Pose2d's as desired
        Pose2d initPose = new Pose2d(0, 0, 0); // Starting Pose
        Pose2d dropPurplePixelPose = new Pose2d(0, 0, 0);
        Pose2d midwayPose1 = new Pose2d(0,0,0);
        Pose2d midwayPose2 = new Pose2d(0,0,0);
        Pose2d intakeStack = new Pose2d(0,0,0);
        Pose2d dropYellowPixelPose = new Pose2d(0, 0, 0);
        Pose2d parkPose = new Pose2d(0, 0, 0);

        switch (startPosition) {
            case BLUE_LEFT:
                initPose = new Pose2d(-60, 12, Math.toRadians(0)); //Starting pose
                switch(identifiedSpikeMarkLocation){
                    case LEFT:
                        dropPurplePixelPose = new Pose2d(-35, 12, Math.toRadians(45));
                        dropYellowPixelPose = new Pose2d(-40, 48, Math.toRadians(90));
                        break;
                    case MIDDLE:
                        dropPurplePixelPose = new Pose2d(-32, 15, Math.toRadians(0));
                        dropYellowPixelPose = new Pose2d(-36, 48, Math.toRadians(90));
                        break;
                    case RIGHT:
                        dropPurplePixelPose = new Pose2d(-35, 9, Math.toRadians(-45));
                        dropYellowPixelPose = new Pose2d(-32, 48, Math.toRadians(90));
                        break;
                }
                midwayPose1 = new Pose2d(-50,12,0);
                midwayPose2 = new Pose2d(-50,36, Math.toRadians(90));
                parkPose = new Pose2d(-60,48, Math.toRadians(90));
                break;
            case BLUE_RIGHT:
                initPose = new Pose2d(-60, 36, Math.toRadians(0)); //Starting pose
                switch(identifiedSpikeMarkLocation){
                    case LEFT:
                        dropPurplePixelPose = new Pose2d(-15, -32, Math.toRadians(135));
                        dropYellowPixelPose = new Pose2d(-40, 48, Math.toRadians(90));
                        break;
                    case MIDDLE:
                        dropPurplePixelPose = new Pose2d(-8, -36, Math.toRadians(180));
                        dropYellowPixelPose = new Pose2d(-36, 48, Math.toRadians(90));
                        break;
                    case RIGHT:
                        dropPurplePixelPose = new Pose2d(-15, -40, Math.toRadians(225));
                        dropYellowPixelPose = new Pose2d(-32, 48, Math.toRadians(90));
                        break;
                }
                intakeStack = new Pose2d(-12,-54,Math.toRadians(270));
                midwayPose1 = new Pose2d(-12,-18,90);
                midwayPose2 = new Pose2d(-12,36, Math.toRadians(90));
                parkPose = new Pose2d(-12,46, Math.toRadians(90));
                break;
            case RED_LEFT:
                initPose = new Pose2d(60, 36, Math.toRadians(0)); //Starting pose
                switch(identifiedSpikeMarkLocation){
                    case LEFT:
                        dropPurplePixelPose = new Pose2d(15, -40, Math.toRadians(45));
                        dropYellowPixelPose = new Pose2d(32, 48, Math.toRadians(90));
                        break;
                    case MIDDLE:
                        dropPurplePixelPose = new Pose2d(8, -36, Math.toRadians(0));
                        dropYellowPixelPose = new Pose2d(36, 48, Math.toRadians(90));
                        break;
                    case RIGHT:
                        dropPurplePixelPose = new Pose2d(15, -32, Math.toRadians(-45));
                        dropYellowPixelPose = new Pose2d(40, 48, Math.toRadians(90));
                        break;
                }
                intakeStack = new Pose2d(12,-54,Math.toRadians(270));
                midwayPose1 = new Pose2d(12,-18,90);
                midwayPose2 = new Pose2d(12,36, Math.toRadians(90));
                parkPose = new Pose2d(12,46, Math.toRadians(90));
                break;
            case RED_RIGHT:
                initPose = new Pose2d(60, 12, Math.toRadians(180)); //Starting pose
                switch(identifiedSpikeMarkLocation){
                    case LEFT:
                        dropPurplePixelPose = new Pose2d(35, 9, Math.toRadians(135));
                        dropYellowPixelPose = new Pose2d(32, 48, Math.toRadians(90));
                        break;
                    case MIDDLE:
                        dropPurplePixelPose = new Pose2d(32, 12, Math.toRadians(180));
                        dropYellowPixelPose = new Pose2d(36, 48, Math.toRadians(90));
                        break;
                    case RIGHT:
                        dropPurplePixelPose = new Pose2d(35, 15, Math.toRadians(225));
                        dropYellowPixelPose = new Pose2d(40, 48, Math.toRadians(90));
                        break;
                }
                midwayPose1 = new Pose2d(50,12,180);
                midwayPose2 = new Pose2d(50,36, Math.toRadians(90));
                parkPose = new Pose2d(60,48, Math.toRadians(90));
                break;
        }

        drive.pose = initPose;

        //Move robot to dropPurplePixel based on identified Spike Mark Location
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(dropPurplePixelPose.position, dropPurplePixelPose.heading)
                        //.splineToLinearHeading(dropPurplePixelPose,0)
                        .build());

        //TODO : Code action to drop Purple Pixel on Spike Mark

        //For Blue Right and Red Left, intake pixel from stack
        if (startPosition == START_POSITION.BLUE_RIGHT ||
            startPosition == START_POSITION.RED_LEFT) {
            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .strafeToLinearHeading(intakeStack.position, intakeStack.heading)
                            //.splineToLinearHeading(dropPurplePixelPose,0)
                            .build());
        }

        //TODO : Code action to intake pixel from stack

        //Move robot through midwayPose1, interimPose 2 to dropYellowPixelPose
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(midwayPose1.position, midwayPose1.heading)
                        .strafeToLinearHeading(midwayPose2.position, midwayPose2.heading)
                        //.splineToLinearHeading(midwayPose1,0)
                        //.splineToLinearHeading(midwayPose2,0)
                        .waitSeconds(5) //TODO: Adjust time to wait for alliance partner to move from board
                        .splineToLinearHeading(dropYellowPixelPose,0)
                        .build());

        //TODO : Code action to drop Pixel

        //Move robot to parking
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(parkPose.position, parkPose.heading)
                        //.splineToLinearHeading(parkPose,0)
                        .build());
    }


    //Method to select starting position using X, Y, A, B buttons on gamepad
    public void selectStartingPosition() {
        telemetry.setAutoClear(true);
        telemetry.clearAll();
        //******select start pose*****
        while(!isStopRequested()){
            telemetry.addData("Initializing FTC Wires (ftcwires.org) Autonomous adopted for Team:","TEAM NUMBER");
            telemetry.addData("---------------------------------------","");
            telemetry.addData("Select Starting Position using XYAB Keys on gamepad 1:","");
            telemetry.addData("    Blue Left   ", "(X)");
            telemetry.addData("    Blue Right ", "(Y)");
            telemetry.addData("    Red Left    ", "(B)");
            telemetry.addData("    Red Right  ", "(A)");
            if(gamepad1.x){
                startPosition = START_POSITION.BLUE_LEFT;
                break;
            }
            if(gamepad1.y){
                startPosition = START_POSITION.BLUE_RIGHT;
                break;
            }
            if(gamepad1.b){
                startPosition = START_POSITION.RED_LEFT;
                break;
            }
            if(gamepad1.a){
                startPosition = START_POSITION.RED_RIGHT;
                break;
            }
            telemetry.update();
        }
        telemetry.clearAll();
    }



    /**
     * Initialize the TensorFlow Object Detection processor.
     */
    private void initTfod() {

        // Create the TensorFlow processor the easy way.
        tfod = TfodProcessor.easyCreateWithDefaults();

        // Create the vision portal the easy way.
        if (USE_WEBCAM) {
            visionPortal = VisionPortal.easyCreateWithDefaults(
                hardwareMap.get(WebcamName.class, "Webcam 1"), tfod);
        } else {
            visionPortal = VisionPortal.easyCreateWithDefaults(
                BuiltinCameraDirection.BACK, tfod);
        }

    }   // end method initTfod()

    /**
     * Add telemetry about TensorFlow Object Detection (TFOD) recognitions.
     */
    private void runTfodTensorFlow() {

        List<Recognition> currentRecognitions = tfod.getRecognitions();
        telemetry.addData("# Objects Detected", currentRecognitions.size());

        // Step through the list of recognitions and display info for each one.
        for (Recognition recognition : currentRecognitions) {
            double x = (recognition.getLeft() + recognition.getRight()) / 2 ;
            double y = (recognition.getTop()  + recognition.getBottom()) / 2 ;

            telemetry.addData(""," ");
            telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
            telemetry.addData("- Position", "%.0f / %.0f", x, y);
            telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());

            if (x < 100) {
                identifiedSpikeMarkLocation = IDENTIFIED_SPIKE_MARK_LOCATION.LEFT;
            } else if (x>100 && x <200) {
                identifiedSpikeMarkLocation = IDENTIFIED_SPIKE_MARK_LOCATION.MIDDLE;
            } else { //x > 200
                identifiedSpikeMarkLocation = IDENTIFIED_SPIKE_MARK_LOCATION.RIGHT;
            }

        }   // end for() loop

    }   // end method runTfodTensorFlow()

}   // end class
