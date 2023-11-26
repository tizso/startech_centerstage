package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class HardwareBox {
    public DcMotor colector = null;
    public DcMotor arm = null;
    public Servo clawLeft = null;
    public Servo clawRight = null;
    public Servo clawArm = null;

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

        // Set all motors to zero power
        arm.setPower(0.0);
        colector.setPower(0.0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        colector.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Define and initialize ALL installed servos.
        clawLeft  = hwMap.get(Servo.class, "clawLeft");
        clawRight  = hwMap.get(Servo.class, "clawRight");
        clawArm  = hwMap.get(Servo.class, "clawArm");
        clawArm.setPosition(0.38);
        clawLeft.setPosition(0.4);
        clawRight.setPosition(0.6);

    }

}
