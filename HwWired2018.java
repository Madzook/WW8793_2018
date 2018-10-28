package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * This is a hardware class for the Wired Woodman
 *
 * Revised 13Feb2018 - Modified Glyph Arm speed and limits; set glyph arm zero power behavior to BRAKE
 * Revised 15Feb2018 - Removed glyph arm ZeroPowerBehavior
 */

public class HwWired2018
{
    /* Public OpMode members. */
    DcMotor
            leftDrive = null,
            rightDrive = null,
            latchArm = null;

    Servo
             latchArmGripper = null,
            frontExtension = null,
            backExtension = null;

    final double
            //Numbers need changed, these are servos
            L_GRIPPER_OPEN =  0.1,
            L_GRIPPER_CLOSED =  0.75,
            F_EXTEND_OUT = 0.8,
            F_EXTEND_IN = 0.5,
            B_EXTEND_OUT = .8,
            B_EXTEND_IN = 0.5,

            // These ones are for the motors
            L_ARM_SPEED_FACTOR = 0.2;



    final int
            // These numbers are motors
            G_ARM_UP_POS =  1250,
            G_ARM_SAFE_POS = 725,
            G_ARM_DOWN_POS =  0,
            R_ARM_UP_POS =  17000,
            R_ARM_SAFE_POS = 400,
            R_ARM_DOWN_POS = 0,
            R_ARM_OUT_POS = 8550,
            R_ARM_IN_POS = 0;

    /* local OpMode members. */
    HardwareMap hwMap =  null;


    /* Constructor */
    public HwWired2018(){
    }


    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {

        // Save reference to Hardware map
        hwMap = ahwMap;


        // Identify and link connected hardware
        leftDrive   = hwMap.dcMotor.get("left_drive");
        rightDrive  = hwMap.dcMotor.get("right_drive");
        latchArm    = hwMap.dcMotor.get("latch_arm");


        latchArmGripper = hwMap.servo.get("latch_gripper");
        frontExtension = hwMap.servo.get("front_extend");
        backExtension =  hwMap.servo.get("back_extend");



        // Configure and Initialize Motors
        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        rightDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        latchArm.setDirection(DcMotor.Direction.REVERSE);
        latchArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        latchArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);



        // Set all motors to zero power
        leftDrive.setPower(0);
        rightDrive.setPower(0);
        latchArm.setPower(0);


        // Initialize servo positions.
        latchArmGripper.setPosition(L_GRIPPER_CLOSED);
        frontExtension.setPosition(F_EXTEND_IN);
        backExtension.setPosition(B_EXTEND_OUT);
    }

}

