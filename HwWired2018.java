package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * This is a hardware class for the Wired Woodman
 *
 * Revised 13Feb2018 by Josh Sirkin and Mike Zook - Modified Glyph Arm speed and limits; set glyph arm zero power behavior to BRAKE
 * Revised 16January2019 by Blake Roscoe - Changed numbers of positions for mineral arm
 */

public class HwWired2018
{
    /* Public OpMode members. */
    DcMotor
            leftDrive = null,
            rightDrive = null,
            latchArm = null,
            mineralMove = null,
            mineralCollector = null;


    Servo
            latchArmGripper = null,
            frontExtension = null,
            backExtension = null,
            mineralStomper = null;


    final double
            //Numbers need changed, these are servos
            L_GRIPPER_OPEN =  0,
            L_GRIPPER_CLOSED =  1,
            F_EXTEND_OUT = .7,
            F_RETRACT = 0,
            B_EXTEND_OUT = .7,
            B_RETRACT = 0,
            M_STOMP_UP = .8,
            M_STOMP_DOWN = .05,
            M_STOMP_INC = 0.015,
    
// These ones are for the motors
            L_ARM_SPEED_FACTOR = 1.0,
            M_MOVE_SPEED_FACTOR = .4,
            M_COLLECTOR_SPEED_FACTOR = .4;


    final int
            // These numbers are motors
            /*Gear ratio=(120/80)(120/40)
            Encoder = 1680
            Mineral Encoder = 1440
            Position = (Distance) (1/diameter) (Encoder count/1)
            Our formula = (15 in.) (1/.869 in.) (1680/1)
            For mineral arm = ((degrees) (encoder count) (gear ratio)) /360*/
            L_ARM_UP_POS = 0,
            L_ARM_DOWN_POS = -18000,
            M_MOVE_UP_POS = 2100,
            M_MOVE_DOWN_POS = 0,
            M_COLLECTOR_UP_POS = 3500,
            M_COLLECTOR_DOWN_POS = 0;




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
        mineralMove = hwMap.dcMotor.get("mineral_move");
        mineralCollector = hwMap.dcMotor.get("mineral_collector");



        latchArmGripper = hwMap.servo.get("latch_gripper");
        frontExtension = hwMap.servo.get("front_extend");
        backExtension =  hwMap.servo.get("back_extend");
        mineralStomper = hwMap.servo.get("mineral_stomper");



        // Configure and Initialize Motors
        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        rightDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        latchArm.setDirection(DcMotor.Direction.REVERSE);
        latchArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        latchArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        mineralMove.setDirection(DcMotor.Direction.FORWARD);
        mineralMove.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mineralMove.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        mineralCollector.setDirection(DcMotor.Direction.REVERSE);
        mineralCollector.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mineralCollector.setMode(DcMotor.RunMode.RUN_TO_POSITION);



        // Set all motors to zero power
        leftDrive.setPower(0);
        rightDrive.setPower(0);
        latchArm.setPower(0);
        mineralMove.setPower(0);
        mineralCollector.setPower(0);


        // Initialize servo positions.
        latchArmGripper.setPosition(L_GRIPPER_CLOSED);
        frontExtension.setPosition(F_RETRACT);
        backExtension.setPosition(B_EXTEND_OUT);
        mineralStomper.setPosition(M_STOMP_DOWN);

    }

}

