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
    int hangDown = 0;

    @Override

    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);

        double SLOW_DOWN_FACTOR = 0.9; //TODO Adjust to driver comfort
        telemetry.addData("Initializing FTC TeleOp adopted for Team:","18338");
        telemetry.update();

        robot.arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.hangUp.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.arm.setDirection(DcMotorEx.Direction.FORWARD);
        robot.arm.setTargetPosition(20);
        robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.arm.setPower(0.9);
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

            double armSpeed = 0.7;

            if(gamepad1.dpad_up && up == 0){
                robot.arm.setDirection(DcMotorEx.Direction.FORWARD);
                robot.arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                robot.arm.setPower(armSpeed/4);
                up=1;
                sleep(200);
            }

            if(gamepad1.dpad_up && up == 1){
                robot.arm.setPower(0);
                up = 0;
                sleep(200);
            }

            if(gamepad1.dpad_down & down == 0){
                robot.arm.setDirection(DcMotorSimple.Direction.REVERSE);
                robot.arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                robot.arm.setPower(armSpeed/4);
                down = 1;
                sleep(200);
            }

            if(gamepad1.dpad_down & down == 1){
                robot.arm.setPower(0);
                down = 0;
                sleep(200);
            }

            if((gamepad1.y || gamepad1.x) && robot.colector.getPower() == 0){
                robot.colector.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                robot.colector.setDirection(gamepad1.x?DcMotorEx.Direction.FORWARD:DcMotorEx.Direction.REVERSE);
                robot.colector.setPower(0.7);
                sleep(200);
            }
            if((gamepad1.y || gamepad1.x) && robot.colector.getPower() > 0){
                robot.colector.setPower(0);
                sleep(200);
            }

            if (gamepad1.left_bumper && suportUp == 0) {
                robot.suport.setPosition(1);
                sleep(300);
                suportUp = 1;
            }

            if (gamepad1.left_bumper && suportUp == 1) {
                robot.suport.setPosition(0);
                sleep(300);
                suportUp = 0;
            }

            if (gamepad1.right_bumper) {
                robot.hangUp.setDirection(DcMotorEx.Direction.FORWARD);
                robot.hangUp.setTargetPosition(17000);
                robot.hangUp.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.hangUp.setPower(0.9);
                sleep(300);

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
                robot.closeClaw();

                robot.arm.setDirection(DcMotorEx.Direction.FORWARD);
                robot.arm.setTargetPosition(2500);
                robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.arm.setPower(armSpeed);
                robot.clawArm.setPosition(0.5);
                sleep(200);
            }
            if(gamepad2.b){
                robot.closeClaw();

                robot.arm.setDirection(DcMotorEx.Direction.FORWARD);
                robot.arm.setTargetPosition(2000);
                robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.arm.setPower(armSpeed);
                robot.clawArm.setPosition(0.4);
                sleep(200);
            }
            if(gamepad2.x){
                robot.closeClaw();

                robot.arm.setDirection(DcMotorEx.Direction.FORWARD);
                robot.arm.setTargetPosition(1500);
                robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.arm.setPower(armSpeed);
                robot.clawArm.setPosition(0.1);
                sleep(200);
            }

            if(gamepad2.back){
                robot.clawArm.setPosition(0.36);
                robot.closeClaw();
                robot.arm.setDirection(DcMotorEx.Direction.FORWARD);
                robot.arm.setTargetPosition(20);
                robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.arm.setPower(armSpeed);
                sleep(200);
            }

            if(gamepad2.a){
                robot.clawArm.setPosition(0.36);
                robot.closeClaw();
                robot.arm.setDirection(DcMotorEx.Direction.FORWARD);
                robot.arm.setTargetPosition(20);
                robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.arm.setPower(armSpeed);
                sleep(200);
            }

            if (gamepad2.right_bumper) {
                robot.openClaw();
                sleep(300);
            }

            if (gamepad2.left_bumper) {
                robot.closeClaw();
                sleep(300);
            }

            telemetry.addLine("Current Pose");
            telemetry.addData("x", drive.pose.position.x);
            telemetry.addData("y", drive.pose.position.y);
            telemetry.addData("heading", Math.toDegrees(drive.pose.heading.log()));
            telemetry.addData("arm position",robot.arm.getCurrentPosition());
            telemetry.addData("clawArm", robot.clawArm.getPosition());
            telemetry.addData("clawLeft", robot.clawLeft.getPosition());
            telemetry.addData("clawRight", robot.clawRight.getPosition());
            telemetry.addData("dpad_down", gamepad1.dpad_down);
            telemetry.addData("colector get dir: ", robot.colector.getDirection());
            telemetry.addData("hangUp: ", robot.hangUp.getCurrentPosition());

            telemetry.update();
        }
    }

}