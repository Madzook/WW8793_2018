/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Primary Wired Woodman Teleop Program
 *
 * Revised 05December2018 - Added mineral collector and mover
 * Revised 07January2019 - Added mineral stomper
 */

@TeleOp(name="Wired Woodman", group="Wired")
//@Disabled
public class WiredTeleopIterative extends OpMode{

    /* Declare OpMode members. */
    HwWired2018 robot = new HwWired2018(); // New Robot object created from the HwWired2018 class

    private double
            leftDrive = 0,
            rightDrive = 0,
            latchArm = 0,
            latchArmSpeed = 0,
            mineralMoveSpeed = 0,
            mineralCollectorMoveSpeed = 0,
            mineralStomperPosition = robot.M_STOMP_UP;


    private boolean
            moveLatchArm = false,
            moveMineralMove = false,
            moveMineralCollector = false;


    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {

        robot.init(hardwareMap); // Initialize the robot object via it's init method
        robot.leftDrive.setPower(0);
        robot.rightDrive.setPower(0);
        robot.latchArm.setPower(0);
        robot.mineralMove.setPower(0);
        robot.mineralCollector.setPower(0);

        telemetry.addLine("Say Hello Driver");  // Send message to signify robot waiting
    }


    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }


    /*
     * Code to run ONCE when the driver hits PLAY
     */



    @Override
    public void start() {
    }


    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {

        // Run wheels in tank mode (note: The joystick goes negative when pushed forwards, so negate it)
        leftDrive = -gamepad1.left_stick_y;
        rightDrive = -gamepad1.right_stick_y;
        robot.leftDrive.setPower(leftDrive);
        robot.rightDrive.setPower(rightDrive);



        /* This is an alternative to moving the latch arm with triggers
        latchArm = -gamepad2.left_stick_y + -gamepad2.right_stick_y;
        robot.latchArm.setPower(latchArm); */



        // Gamepad 1 controls
        // Raise/lower latch arm with left/right triggers
        if (gamepad1.left_trigger >= 0.10) {
            robot.latchArm.setTargetPosition(robot.L_ARM_UP_POS);
            latchArmSpeed = gamepad1.left_trigger * robot.L_ARM_SPEED_FACTOR;
            robot.latchArm.setPower(latchArmSpeed);
            moveLatchArm = true;
        }

        if (gamepad1.right_trigger >= 0.10) {
            robot.latchArm.setTargetPosition(robot.L_ARM_DOWN_POS);
            latchArmSpeed = gamepad1.right_trigger * robot.L_ARM_SPEED_FACTOR;
            robot.latchArm.setPower(latchArmSpeed);
            moveLatchArm = true;
        }
        if (gamepad1.right_trigger < 0.10 && gamepad1.left_trigger < 0.10 && moveLatchArm) {
            robot.latchArm.setTargetPosition(robot.latchArm.getCurrentPosition());
            robot.latchArm.setPower(0.1);
            moveLatchArm = false;
        }

        if (gamepad1.left_bumper) {
            robot.latchArmGripper.setPosition(robot.L_GRIPPER_OPEN);
        }

        if (gamepad1.right_bumper) {
            robot.latchArmGripper.setPosition(robot.L_GRIPPER_CLOSED);
        }

        if (gamepad1.dpad_left) {
            robot.frontExtension.setPosition(robot.F_RETRACT);
            robot.backExtension.setPosition(robot.B_RETRACT);

        }

        if (gamepad1.dpad_right) {
            robot.frontExtension.setPosition(robot.F_EXTEND_OUT);
            robot.backExtension.setPosition(robot.B_EXTEND_OUT);
        }


        //Gamepad 2 controls
        //Moving the bottom mineral arm motor
        if (gamepad2.left_trigger >= 0.10 ) {
            robot.mineralMove.setTargetPosition(robot.M_MOVE_UP_POS);
            mineralMoveSpeed = gamepad2.left_trigger * robot.M_MOVE_SPEED_FACTOR;
            robot.mineralMove.setPower(mineralMoveSpeed);
            moveMineralMove = true;
            }

        if (gamepad2.right_trigger >= 0.10 )  {
            robot.mineralMove.setTargetPosition(robot.M_MOVE_DOWN_POS);
            mineralMoveSpeed = gamepad2.right_trigger * robot.M_MOVE_SPEED_FACTOR;
            robot.mineralMove.setPower(mineralMoveSpeed);
            moveMineralMove = true;
        }

        if (gamepad2.left_trigger < 0.10 && gamepad2.right_trigger < 0.10 && moveMineralMove) {
            robot.mineralMove.setTargetPosition(robot.mineralMove.getCurrentPosition());
            robot.mineralMove.setPower(0.1);
            moveMineralMove = false;
        }

        //Moving the middle motor on the mineral arm
        if (gamepad2.left_bumper)   {
            robot.mineralCollector.setTargetPosition(robot.M_COLLECTOR_UP_POS);
            mineralCollectorMoveSpeed = robot.M_COLLECTOR_SPEED_FACTOR;
            robot.mineralCollector.setPower(mineralCollectorMoveSpeed);
            moveMineralCollector = true;
        }

        if (gamepad2.right_bumper)  {
            robot.mineralCollector.setTargetPosition(robot.M_COLLECTOR_DOWN_POS);
            mineralCollectorMoveSpeed = robot.M_COLLECTOR_SPEED_FACTOR;
            robot.mineralCollector.setPower(mineralCollectorMoveSpeed);
            moveMineralCollector = true;

        }

        if (!gamepad2.left_bumper && !gamepad2.right_bumper && moveMineralCollector) {
            robot.mineralCollector.setTargetPosition(robot.mineralCollector.getCurrentPosition());
            mineralCollectorMoveSpeed = 0.1;
            robot.mineralCollector.setPower(mineralCollectorMoveSpeed);
            moveMineralCollector = false;
        }

        //Moving the mineral servo up and down
        if (gamepad2.y) {
            mineralStomperPosition = mineralStomperPosition + robot.M_STOMP_INC;
            if (mineralStomperPosition > robot.M_STOMP_UP)
                mineralStomperPosition = robot.M_STOMP_UP;
            robot.mineralStomper.setPosition(mineralStomperPosition);
        }

        if (gamepad2.a) {
            mineralStomperPosition = mineralStomperPosition - robot.M_STOMP_INC;
            if (mineralStomperPosition < robot.M_STOMP_DOWN)
                mineralStomperPosition = robot.M_STOMP_DOWN;
            robot.mineralStomper.setPosition(mineralStomperPosition);
        }

        //Putting all motors and servo in compact position
        if (gamepad2.x) {
            robot.mineralStomper.setPosition(robot.M_STOMP_DOWN);
            robot.mineralCollector.setTargetPosition(robot.M_COLLECTOR_DOWN_POS);
            robot.mineralCollector.setTargetPosition(robot.M_MOVE_DOWN_POS);
            moveMineralCollector = false;
            moveMineralMove = false;
        }



        // Provide critical data to the Driver Station phone
        telemetry.addData("left",  "%.2f", leftDrive);
        telemetry.addData("right", "%.2f", rightDrive);
        telemetry.addData("Left Position", robot.leftDrive.getCurrentPosition());
        telemetry.addData("Right Position", robot.rightDrive.getCurrentPosition());
        telemetry.addData("L Gripper", "%.2f" ,robot.latchArmGripper.getPosition());
        telemetry.addData("G1 Left Bumper", gamepad1.left_bumper);
        telemetry.addData("G1 Right Bumper", gamepad1.right_bumper);
        telemetry.addData("L Arm Position", robot.latchArm.getCurrentPosition());
        telemetry.addData("Mineral Move Position", robot.mineralMove.getCurrentPosition());
        telemetry.addData("Mineral Collector Position", robot.mineralCollector.getCurrentPosition() );
        telemetry.addData("Mineral Collector Set Speed", mineralCollectorMoveSpeed);
        telemetry.addData("G2 Left Bumper", gamepad2.left_bumper);
        telemetry.addData("G2 Right Bumper", gamepad2.right_bumper);
        telemetry.addData("M Stomper", "%.2f" ,mineralStomperPosition);
        telemetry.addData("Back Extension", "%.2f" ,robot.backExtension.getPosition());
        telemetry.addData("Front Extension", "%.2f" ,robot.frontExtension.getPosition());
    }


    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {

        // Set all motors to zero power
        robot.leftDrive.setPower(0);
        robot.rightDrive.setPower(0);
        robot.latchArm.setPower(0);
        robot.mineralCollector.setPower(0);
        robot.mineralMove.setPower(0);

    }

}
