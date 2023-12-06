package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class HardwareBox {
    public DcMotor colector = null;
    public DcMotor arm = null;
    public DcMotor hangUp = null;
    public Servo clawLeft = null;
    public Servo clawRight = null;
    public Servo clawArm = null;

    public Servo suport = null;

    HardwareMap hwMap		   =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public HardwareBox(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        arm   = hwMap.get(DcMotor.class, "arm");
        colector    = hwMap.get(DcMotor.class, "colector");
        hangUp = hwMap.get(DcMotor.class, "hangUp");

        // Set all motors to zero power
        arm.setPower(0.0);
        colector.setPower(0.0);
        hangUp.setPower(0.0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        colector.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        hangUp.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Define and initialize ALL installed servos.
        clawLeft  = hwMap.get(Servo.class, "clawLeft");
        clawRight  = hwMap.get(Servo.class, "clawRight");
        clawArm  = hwMap.get(Servo.class, "clawArm");
        suport = hwMap.get(Servo.class, "suport");
        clawArm.setPosition(0.36);
        clawLeft.setPosition(0.40);
        clawRight.setPosition(0.55);
        suport.setPosition(0);

    }

    public void closeClaw(){
        clawLeft.setPosition(0.40);
        clawRight.setPosition(0.55);
    }

    public void openClaw(){
        clawLeft.setPosition(0.55);
        clawRight.setPosition(0.40);
    }

}
