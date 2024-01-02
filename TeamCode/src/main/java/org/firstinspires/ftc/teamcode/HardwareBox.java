package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.util.ElapsedTime.Resolution.SECONDS;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class HardwareBox extends LinearOpMode{
    //public NormalizedColorSensor color;

    public DcMotor colector = null;
    public DcMotor intake = null;
    public DcMotor slider = null;
    public DcMotor hangUp = null;
    public Servo leftS = null;
    public Servo rightS = null;
    public Servo pixelS = null;

    public Servo suport = null;

    HardwareMap hwMap		   =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public HardwareBox(){

    }
    public void runOpMode(){}

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        slider = hwMap.get(DcMotor.class, "slider");
        colector    = hwMap.get(DcMotor.class, "colector");
        intake    = hwMap.get(DcMotor.class, "intake");
        hangUp = hwMap.get(DcMotor.class, "hangUp");

        //color = hardwareMap.get(NormalizedColorSensor.class, "color");

        // Set all motors to zero power
        slider.setPower(0.0);
        colector.setPower(0.0);
        intake.setPower(0.0);
        hangUp.setPower(0.0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        slider.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        colector.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        hangUp.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Define and initialize ALL installed servos.
        leftS = hwMap.get(Servo.class, "leftS");
        rightS = hwMap.get(Servo.class, "rightS");
        pixelS = hwMap.get(Servo.class, "pixelS");
        suport = hwMap.get(Servo.class, "suport");
        pixelS.setPosition(0.36);
        leftS.setPosition(0.0);
        rightS.setPosition(1.0);
        suport.setPosition(0);

    }

    public void getPixel(){
        leftS.setPosition(0.0);
        rightS.setPosition(1.0);
    }

    public void putPixel(){
        leftS.setPosition(1.0);
        rightS.setPosition(0.0);
    }

    public void safeWaitSeconds(double time) {
        ElapsedTime timer = new ElapsedTime(SECONDS);
        timer.reset();
        while (!isStopRequested() && timer.time() < time) {
        }
    }

    public void putPurplePixel(){
        safeWaitSeconds(1);
        colector.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        colector.setDirection(DcMotorEx.Direction.REVERSE);
        colector.setPower(0.3);
        safeWaitSeconds(0.7);
        colector.setPower(0);
        safeWaitSeconds(1);
    }

    public void sliderUp(){
        slider.setDirection(DcMotorEx.Direction.FORWARD);
        slider.setTargetPosition(200);
        slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slider.setPower(0.7);

        safeWaitSeconds(1);
        putPixel();
        safeWaitSeconds(1);
    }

    public void dropPixel(){
        safeWaitSeconds(1);

        pixelS.setPosition(0.6);

        safeWaitSeconds(1);
        getPixel();
        pixelS.setPosition(0.36);

        slider.setDirection(DcMotorEx.Direction.FORWARD);
        slider.setTargetPosition(0);
        slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slider.setPower(0.7);
    }

}
