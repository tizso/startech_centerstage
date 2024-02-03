package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


/**
 * FTC WIRES TeleOp Example
 *
 */

@TeleOp(name = "TeleOp StarTech", group = "00-Teleop")
public class FTCWiresTeleOpMode extends LinearOpMode {

    HardwareBox robot = new HardwareBox();

    int up = 0;
    int down = 0;
    int suportUp = 0;
    int hangUp = 0;
    int dronePos = 0;

    @Override

    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);

        double SLOW_DOWN_FACTOR = 0.9; //TODO Adjust to driver comfort
        telemetry.addData("Initializing FTC TeleOp adopted for Team:","18338");
        telemetry.update();

        robot.slider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.hangUp.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);



        robot.slider.setDirection(DcMotorEx.Direction.FORWARD);
        robot.slider.setTargetPosition(0);
        robot.slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.slider.setPower(0.9);
        sleep(200);

        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        waitForStart();

        while (opModeIsActive()) {
            if(gamepad1.b ){
                SLOW_DOWN_FACTOR = 0.3;
            }
            if(gamepad1.a ){
                SLOW_DOWN_FACTOR = 0.9;
            }

            telemetry.addData("Running StarTech TeleOp Mode adopted for Team:","18338");
            drive.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(
                            -gamepad1.left_stick_y * SLOW_DOWN_FACTOR,
                            -gamepad1.left_stick_x * SLOW_DOWN_FACTOR
                    ),
                    -gamepad1.right_stick_x * SLOW_DOWN_FACTOR
            ));

            drive.updatePoseEstimate();

            double sliderSpeed = 0.7;

            /*if(gamepad1.dpad_up && up == 0){
                robot.slider.setDirection(DcMotorEx.Direction.FORWARD);
                robot.slider.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                robot.slider.setPower(sliderSpeed/4);
                up=1;
                sleep(200);
            }

            if(gamepad1.dpad_up && up == 1){
                robot.slider.setPower(0);
                up = 0;
                sleep(200);
            }

            if(gamepad1.dpad_down & down == 0){
                robot.slider.setDirection(DcMotorSimple.Direction.REVERSE);
                robot.slider.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                robot.slider.setPower(sliderSpeed/4);
                down = 1;
                sleep(200);
            }

            if(gamepad1.dpad_down & down == 1){
                robot.slider.setPower(0);
                down = 0;
                sleep(200);
            }
*/
            if((gamepad1.y || gamepad1.x) && robot.colector.getPower() == 0){
                robot.colector.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                robot.colector.setDirection(gamepad1.x?DcMotorEx.Direction.FORWARD:DcMotorEx.Direction.REVERSE);
                robot.colector.setPower(0.5);

                robot.intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                robot.intake.setDirection(gamepad1.x?DcMotorEx.Direction.REVERSE:DcMotorEx.Direction.FORWARD);
                robot.intake.setPower(1.0);
                sleep(200);
            }
            if((gamepad1.y || gamepad1.x) && robot.colector.getPower() > 0){
                robot.colector.setPower(0);
                robot.intake.setPower(0);
                sleep(200);
            }

            if (gamepad1.left_bumper && suportUp == 0) {
                robot.suport.setPosition(0.5);
                sleep(300);
                suportUp = 1;

            }

            if (gamepad1.left_bumper && suportUp == 1) {
                robot.suport.setPosition(0);
                sleep(300);
                suportUp = 0;
                hangUp = 1;
            }

            if (gamepad1.right_bumper && hangUp == 1) {
                robot.hangUp.setDirection(DcMotorEx.Direction.FORWARD);
                robot.hangUp.setTargetPosition(11600);
                robot.hangUp.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.hangUp.setPower(0.9);
                sleep(300);
                hangUp = 0;
            }
            /*if (gamepad1.right_bumper && hangUp == 1) {
                robot.hangUp.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                robot.hangUp.setPower(0);
                sleep(300);
                hangUp = 0;
            }*/

            if(gamepad1.back){
                robot.hangUp.setDirection(DcMotorEx.Direction.REVERSE);
                robot.hangUp.setTargetPosition(0);
                robot.hangUp.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.hangUp.setPower(0.5);
                sleep(300);
            }
            
            if(gamepad2.y){
                robot.slider.setDirection(DcMotorEx.Direction.FORWARD);
                robot.slider.setTargetPosition(980);
                robot.slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.slider.setPower(sliderSpeed);
                robot.putPixel();
                sleep(200);
            }
            if(gamepad2.b){
                robot.slider.setDirection(DcMotorEx.Direction.FORWARD);
                robot.slider.setTargetPosition(450);
                robot.slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.slider.setPower(sliderSpeed);
                robot.putPixel();
                sleep(200);
            }
            if(gamepad2.x){
                robot.slider.setDirection(DcMotorEx.Direction.FORWARD);
                robot.slider.setTargetPosition(200);
                robot.slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.slider.setPower(sliderSpeed);
                robot.putPixel();
                sleep(200);
            }

            if(gamepad2.back && dronePos==0){
                dronePos = 1;
                sleep(200);
            }

            if(gamepad2.a){
                robot.pixelS.setPosition(0.36);
                robot.getPixel();
                robot.slider.setDirection(DcMotorEx.Direction.FORWARD);
                robot.slider.setTargetPosition(0);
                robot.slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.slider.setPower(sliderSpeed);
                sleep(200);
            }

            if (gamepad2.right_bumper) {
                robot.pixelS.setPosition(0.7);
                sleep(300);
            }

            if (gamepad2.left_bumper && dronePos==1) {
                robot.drone.setPosition(1);
                dronePos = 0;
                sleep(300);
            }



            telemetry.addLine("Current Pose");
            telemetry.addData("x", drive.pose.position.x);
            telemetry.addData("y", drive.pose.position.y);
            telemetry.addData("heading", Math.toDegrees(drive.pose.heading.log()));
            telemetry.addData("slider position",robot.slider.getCurrentPosition());
            telemetry.addData("clawArm", robot.pixelS.getPosition());
            telemetry.addData("clawLeft", robot.leftS.getPosition());
            telemetry.addData("clawRight", robot.rightS.getPosition());
            telemetry.addData("dpad_down", gamepad1.dpad_down);
            telemetry.addData("colector get dir: ", robot.colector.getDirection());
            telemetry.addData("hangUp: ", robot.hangUp.getCurrentPosition());

            telemetry.update();
        }
    }

}