/* Copyright (c) 2017 FIRST. All rights reserved.
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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Abstract Linear Autonomous OpMode
 * Created 17Oct2018 for the Greenwood FTC Robotics Club.
 * This class provides the basis for all roverRuckus autonomous programs
 */


public abstract class AutoLinearAbstract_2018 extends LinearOpMode {

    /* =======================================================
     * CLASS MEMBERS (i.e., Class Status)
     * Common autonomous opmode members
     * ======================================================= */

    /* -------------------------------------------------------
     * Public (Shared) Class Members
     * Automated objects, timers, variables, constants
     * ------------------------------------------------------- */

    // OBJECTS
    AssemblyDriveTrain
            driveTrain;

    DeviceTargetMotor
            latchArm;

    DeviceColorSensor
            sampleSensor;


    DeviceTargetServo
            latchArmGripper,
            leftExtension,
            rightExtension;


    ElapsedTime
            generalTimer = new ElapsedTime(), // General/multipurpose timer
            blinkTimer = new ElapsedTime(),
            autoTimer = new ElapsedTime();    // Autonomous timer


    // CONSTANTS
    final int
            LATCH_ARM_TRAVEL_DEGREES = 20;

    final double
            MAX_DRIVE_TRAIN_POSITION_ERROR_INCHES = 0.25,

            DRIVE_TRAIN_PIVOT_SPEED = 0.2,
            DRIVE_TRAIN_DEFAULT_SPEED = 0.5,
            DRIVE_TRAIN_STRAIGHT_SPEED = 0.6,
            LATCH_ARM_DEFAULT_SPEED = 0.3,

            LATCH_ARM_GRIPPER_OPEN_POS = 0.1,
            LATCH_ARM_GRIPPER_CLOSED_POS = 0.9,

            LEFT_RETRACT_POS = 0.1,
            LEFT_EXTEND_POS = 0.9,

            RIGHT_RETRACT_POS = 0.1,
            RIGHT_EXTEND_POS = 0.9,
            EXTENSIONS_INC_PER_SCAN = 0.02,

            BLINK_OFF_PERIOD = 0.3,
            BLINK_ON_PERIOD = 0.3;


    final boolean
            FORWARD = false,
            REVERSE = true,
            LIGHT_ON = true,
            LIGHT_OFF = false;

    boolean
            goldLeft,
            goldCenter,
            goldRight,
            goldFound;

    /* =======================================================
     * CLASS METHODS (i.e., Class Behavior)
     * ======================================================= */

    /* -------------------------------------------------------
     * Method: runOpMode (Overridden Linear OpMode)
     * Purpose: Establish and initialize automated objects, and signal initialization completion
     * ------------------------------------------------------- */
    @Override
    public void runOpMode() {

        /* INITIALIZE ROBOT - ESTABLISH ROBOT OBJECTS */

        // Notify drive station that robot objects are being built
        telemetry.addLine("Wait - Building Robot Objects");
        telemetry.update();

        /* Drive Train constructor: hardwareMap, left motor name, left motor direction, right motor name, right motor direction,
                                    encoder counts per output shaft revolution, gear ratio, wheel radius */
        driveTrain = new AssemblyDriveTrain(hardwareMap,"leftDrive",REVERSE,"rightDrive",FORWARD,1440,1.0,2.0);

        /* Target-Motor constructor: hardwareMap, motor name, motor direction,
                              encoder counts per output shaft revolution, gear ratio, wheel radius */
        latchArm = new DeviceTargetMotor(hardwareMap,"latchArm",REVERSE,1440,1,2);

        /* Color sensor constructor: hardwareMap, sensor name, sensor I2C address */
        sampleSensor = new DeviceColorSensor(hardwareMap,"sampleSensor",0x3c);

        /* Target-Servo constructor: hardwareMap, servo name, initial servo position */
        latchArmGripper = new DeviceTargetServo(hardwareMap,"latchArmGripper", LATCH_ARM_GRIPPER_OPEN_POS);
        leftExtension = new DeviceTargetServo(hardwareMap,"leftExtension", LEFT_RETRACT_POS);
        rightExtension = new DeviceTargetServo(hardwareMap,"rightExtension", RIGHT_RETRACT_POS);


         /* INITIALIZE ROBOT - INITIALIZE ROBOT OBJECTS AND CLASSES*/

        // Notify drive station that robot objects are being initialized
        telemetry.addLine("Wait - Initializing Robot Objects");
        telemetry.update();

         /* Reset encoders and place motors into the 'Run-to-Position' mode
            Note: The initialization calls in the following methods could not be performed in the respective
           object constructors */
        driveTrain.resetEncoders();
        latchArm.resetEncoder();

        /* Lock drive train at current position */
        driveTrain.motorLeft.goToAbsoluteDistance(driveTrain.motorLeft.getPosition(),DRIVE_TRAIN_DEFAULT_SPEED);
        driveTrain.motorRight.goToAbsoluteDistance(driveTrain.motorRight.getPosition(),DRIVE_TRAIN_DEFAULT_SPEED);

        /* Complete color sensor setup
           Note: These initialization methods could not be performed in the respective object constructors */
        sampleSensor.initialize();

        /* Reset the jewel color results */
        goldLeft = false;
        goldRight = false;
        goldCenter = false;
        goldFound = false;

        // Note: Servo initialization is completed in the respective object constructors


        /* INITIALIZE ROBOT - SIGNAL INITIALIZATION COMPLETE */

        // Notify drive station that initialization is wrapping up
        telemetry.addLine("Wait - Initializing Almost Complete");
        telemetry.update();

        // Blink all relicRobot lights/LEDs x2 to acknowledge initialization
        for (int index = 0; index < 2; index++){
            // blink off lights
            blinkTimer.reset();
            sampleSensor.turnOffLight();
            while (blinkTimer.seconds() < BLINK_OFF_PERIOD) {}

            // blink on lights
            blinkTimer.reset();
            sampleSensor.turnOnLight();
            while (blinkTimer.seconds() < BLINK_ON_PERIOD) {}

            // Note: Lights/LEDs will remain on after the FOR loop completes
        }

        // Report initialization complete
        telemetry.addLine("Initialization Complete");
        telemetry.addLine("Hold for Start");
        telemetry.update();


        // WAIT FOR THE GAME TO START (driver presses PLAY)
        waitForStart();

        autoTimer.reset();  // Reset/restart the autotimer


        // GAME STARTED - BEGIN AUTONOMOUS OPERATIONS

    }



    /* -------------------------------------------------------
     * Method: driveTrainTelemetry
     * Purpose: Report the position and speed of the drive train wheels
     * ------------------------------------------------------- */
    void driveTrainTelemetry () {
        telemetry.addLine();
        telemetry.addLine("Left Drive Motor");
        telemetry.addData("  Position in EngUnits","%.2f", driveTrain.motorLeft.getPosition());
        telemetry.addData("  Target in EngUnits","%.2f", driveTrain.motorLeft.targetPosition);
        telemetry.addData("  Position in Counts",driveTrain.motorLeft.targetMotor.getCurrentPosition());
        telemetry.addData("  Target in Counts",driveTrain.motorLeft.targetCount);
        telemetry.addData("  Is Busy",driveTrain.motorLeft.targetMotor.isBusy());
        telemetry.addData("  Speed",driveTrain.leftSpeed);
        telemetry.addLine();
        telemetry.addLine("Right Drive Motor");
        telemetry.addData("  Position in EngUnits","%.2f", driveTrain.motorRight.getPosition());
        telemetry.addData("  Target in EngUnits","%.2f", driveTrain.motorRight.targetPosition);
        telemetry.addData("  Position in Counts",driveTrain.motorRight.targetMotor.getCurrentPosition());
        telemetry.addData("  Target in Counts",driveTrain.motorRight.targetCount);
        telemetry.addData("  Is Busy",driveTrain.motorRight.targetMotor.isBusy());
        telemetry.addData("  Speed",driveTrain.rightSpeed);
    }


    void motorTelemetryDegrees (DeviceTargetMotor motor) {
        telemetry.addLine();
        telemetry.addLine(motor.name);
        telemetry.addData(" Position in Degrees", "%.2f degrees ", motor.getDegrees());
        telemetry.addData(" Position in Counts", motor.targetMotor.getCurrentPosition());
    }


    /*
    void jewelColorTelemetry () {
        telemetry.addLine();
        telemetry.addLine("Left Jewel Color Sensor");
        telemetry.addData("  Hue", "%.2f", sampleSensor.hue);
        telemetry.addData("  Saturation", "%.4f", sampleSensor.saturation);
        telemetry.addData("  Value", "%.4f", sampleSensor.value);
        telemetry.addData("  Red", sampleSensor.colorRed);
        telemetry.addData("  Blue", sampleSensor.colorBlue);
        telemetry.addLine();
        telemetry.addLine("Right Jewel Color Sensor");
        telemetry.addData("  Hue", "%.2f", colorRightJewel.hue);
        telemetry.addData("  Saturation", "%.4f", colorRightJewel.saturation);
        telemetry.addData("  Value", "%.4f", colorRightJewel.value);
        telemetry.addData("  Red", colorRightJewel.colorRed);
        telemetry.addData("  Blue", colorRightJewel.colorBlue);
    }
    */

}
