package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.tuning.TuningOpModes;

/**
 * FTC WIRES TeleOp Example
 *
 */

@TeleOp(name = "TeleOp StarTech", group = "00-Teleop")
public class FTCWiresTeleOpMode extends LinearOpMode {
    private DcMotor arm = null;
    private DcMotor colector = null;
    private Servo clawLeft = null;
    private Servo clawRight = null;
    private Servo clawArm = null;

    int clawPosL = 0;
    int clawPosR = 0;
    int up = 0;
    int down = 0;

    @Override

    public void runOpMode() throws InterruptedException {
        double SLOW_DOWN_FACTOR = 0.9; //TODO Adjust to driver comfort
        telemetry.addData("Initializing FTC TeleOp adopted for Team:","18338");
        telemetry.update();

        arm = hardwareMap.get(DcMotor.class, "arm");
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        colector = hardwareMap.get(DcMotor.class, "colector");

        clawArm = hardwareMap.get(Servo.class, "clawArm");
        clawLeft = hardwareMap.get(Servo.class, "clawLeft");
        clawRight = hardwareMap.get(Servo.class, "clawRight");

        clawArm.setPosition(0.38);
        clawLeft.setPosition(0.4);
        clawRight.setPosition(0.6);
        arm.setDirection(DcMotorEx.Direction.FORWARD);
        arm.setTargetPosition(200);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setPower(0.9);
        sleep(200);

        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        waitForStart();

        while (opModeIsActive()) {
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
                arm.setDirection(DcMotorEx.Direction.FORWARD);
                arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                arm.setPower(armSpeed/4);
                up=1;
                sleep(200);
            }

            if(gamepad1.dpad_up && up == 1){
                arm.setPower(0);
                up = 0;
                sleep(200);
            }

            if(gamepad1.dpad_down & down == 0){
                arm.setDirection(DcMotorSimple.Direction.REVERSE);
                arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                arm.setPower(armSpeed/4);
                down = 1;
                sleep(200);
            }

            if(gamepad1.dpad_down & down == 1){
                arm.setPower(0);
                down = 0;
                sleep(200);
            }

            if((gamepad1.y || gamepad1.x) && colector.getPower() == 0){
                colector.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                colector.setDirection(gamepad1.x?DcMotorEx.Direction.FORWARD:DcMotorEx.Direction.REVERSE);
                colector.setPower(0.7);
                sleep(200);
            }
            if((gamepad1.y || gamepad1.x) && colector.getPower() > 0){
                colector.setPower(0);
                sleep(200);
            }

            if(gamepad2.y){
                arm.setDirection(DcMotorEx.Direction.FORWARD);
                arm.setTargetPosition(2500);
                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                arm.setPower(armSpeed);
                clawArm.setPosition(0.5);
                sleep(200);
            }
            if(gamepad2.b){
                arm.setDirection(DcMotorEx.Direction.FORWARD);
                arm.setTargetPosition(2000);
                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                arm.setPower(armSpeed);
                clawArm.setPosition(0.4);
                sleep(200);
            }
            if(gamepad2.x){
                arm.setDirection(DcMotorEx.Direction.FORWARD);
                arm.setTargetPosition(1500);
                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                arm.setPower(armSpeed);
                clawArm.setPosition(0.36);
                sleep(200);
            }

            if(gamepad2.back){
                arm.setDirection(DcMotorEx.Direction.FORWARD);
                arm.setTargetPosition(100);
                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                arm.setPower(armSpeed);
                sleep(200);
            }

            if(gamepad2.a){
                clawArm.setPosition(0.4);
                clawRight.setPosition(0.8);
                clawLeft.setPosition(0.2);
                arm.setDirection(DcMotorEx.Direction.FORWARD);
                arm.setTargetPosition(100);
                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                arm.setPower(armSpeed);
                sleep(200);
                //arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }
            //clawRight.setPosition(pos < 0.55 ? 0.6 : 0.52)
            /*clawLeft.setPosition(0.4);
            clawRight.setPosition(0.6);*/
            if (gamepad2.right_bumper && clawPosR == 0) {
                clawRight.setPosition(0.48);
                sleep(300);
                clawPosR = 1;
            }
            if (gamepad2.right_bumper && clawPosR == 1) {
                clawRight.setPosition(0.6);
                sleep(300);
                clawPosR = 0;
            }
            //clawLeft.setPosition(pos > 0.45 ? 0.4 : 0.48);
            if (gamepad2.left_bumper && clawPosL == 0) {
                clawLeft.setPosition(0.52);
                sleep(300);
                clawPosL = 1;
            }
            if (gamepad2.left_bumper && clawPosL == 1) {
                clawLeft.setPosition(0.38);
                sleep(300);
                clawPosL = 0;
            }

            telemetry.addLine("Current Pose");
            telemetry.addData("x", drive.pose.position.x);
            telemetry.addData("y", drive.pose.position.y);
            telemetry.addData("heading", Math.toDegrees(drive.pose.heading.log()));
            telemetry.addData("arm position", arm.getCurrentPosition());
            telemetry.addData("clawArm", clawArm.getPosition());
            telemetry.addData("clawLeft", clawLeft.getPosition());
            telemetry.addData("clawRight", clawRight.getPosition());
            telemetry.addData("dpad_down", gamepad1.dpad_down);
            telemetry.addData("colector get dir: ", colector.getDirection());

            telemetry.update();
        }
    }
}