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

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

import java.util.List;

/**
 * Abstract Linear Autonomous OpMode
 * Created 03December2018 for the Greenwood FTC Robotics Club.
 * This class provides the basis for all roverRuckus autonomous programs
 */


public abstract class AutoLinearAbstractCamera2018 extends LinearOpMode {

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
            latchArm,
            mineralMove,
            mineralCollector;


    DeviceColorSensor
            sampleSensor;


    DeviceTargetServo
            latchArmGripper,
            frontExtension,
            backExtension,
            mineralSpinner,
            mineralStomper;

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

            MINERAL_MOVE_UP_POS = 1000,
            MINERAL_MOVE_DOWN_POS = 0,
            MINERAL_MOVE_SAFE_LIMIT = 200,

            MINERAL_COLLECTOR_UP_POS = 1000,
            MINERAL_COLLECTOR_DOWN_POS = 0,
            MINERAL_COLLECTOR_SAFE_LIMIT = 200,

            BACK_RETRACT_POS = 0,
            BACK_EXTEND_POS = 0.7,

            FRONT_RETRACT_POS = 0,
            FRONT_EXTEND_POS = 0.7,
            EXTENSIONS_INC_PER_SCAN = 0.02,

            M_STOMP_UP = .8,
            M_STOMP_DOWN = 0.05,

            M_SPIN_FAST_CW = 1,
            M_SPIN_SLOW_CW = 0.75,
            M_SPIN_STOP = 0.5,
            M_SPIN_SLOW_CCW = 0.25,
            M_SPIN_FAST_CCW = 0,

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
            goldFound,
            mineral_moving;

    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";

    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */
    private static final String VUFORIA_KEY = "ASw6Ddb/////AAABmfeUhNl69UL9kPSl+UCgv3aItfN2PrrJ6HbF2uaJvltQAEwSE3KAcwlZfyOvuh0ALJrQIFjRcdparLjRNFwVyxi1ABI1YXHMliOhYeWVp2TJ3o5WqblwmVwPSDiBShNBjP8TUGtM5jJUeNBX2rfIicwPG5F3VopV926rnCdSEqwwLtFeyGT+2NTpAQ9VSdJEFqaJaNyhLA3ZytIW54o/VHuSNoK2KQVZoCogla5i6acoZhOmJAo/qLqd/RvmNXuHEG/BS7hK8MS4KBTw3oZBjOl8Oo03ZA9ZqkK1RhAaylsDERvYXxlwcWFjkOtQqLJ4lZLg2TdOgh5nG9G3Uvar+zac7RwzkudC/0nF/om21OYT";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the Tensor Flow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;


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
        driveTrain = new AssemblyDriveTrain(hardwareMap,"left_drive",FORWARD,"right_drive",REVERSE,1680,1.0,2.0);

        /* Target-Motor constructor: hardwareMap, motor name, motor direction,
                              encoder counts per output shaft revolution, gear ratio, wheel radius */
        latchArm = new DeviceTargetMotor(hardwareMap,"latch_arm",REVERSE,1680);
        mineralCollector = new DeviceTargetMotor(hardwareMap, "mineral_collector",REVERSE,1680);
        mineralMove = new DeviceTargetMotor(hardwareMap,"mineral_move",FORWARD,1680);

        /* Color sensor constructor: hardwareMap, sensor name, sensor I2C address */
        //sampleSensor = new DeviceColorSensor(hardwareMap,"device_color_sensor",0x3c);

        /* Target-Servo constructor: hardwareMap, servo name, initial servo position */
        latchArmGripper = new DeviceTargetServo(hardwareMap,"latch_gripper", LATCH_ARM_GRIPPER_CLOSED_POS);
        frontExtension = new DeviceTargetServo(hardwareMap,"front_extend", BACK_RETRACT_POS);
        backExtension = new DeviceTargetServo(hardwareMap,"back_extend", FRONT_EXTEND_POS);
        mineralStomper = new DeviceTargetServo(hardwareMap,"mineral_stomper", M_STOMP_DOWN);
        mineralSpinner = new DeviceTargetServo(hardwareMap,"mineral_spinner",M_SPIN_STOP);


         /* INITIALIZE ROBOT - INITIALIZE ROBOT OBJECTS AND CLASSES*/

        // Notify drive station that robot objects are being initialized
        telemetry.addLine("Wait - Initializing Robot Objects");
        telemetry.update();

         /* Reset encoders and place motors into the 'Run-to-Position' mode
            Note: The initialization calls in the following methods could not be performed in the respective
           object constructors */
        driveTrain.resetEncoders();
        latchArm.resetEncoder();
        mineralMove.resetEncoder();
        mineralCollector.resetEncoder();

        /* Lock drive train at current position */
        driveTrain.motorLeft.goToAbsoluteDistance(driveTrain.motorLeft.getPosition(),DRIVE_TRAIN_DEFAULT_SPEED);
        driveTrain.motorRight.goToAbsoluteDistance(driveTrain.motorRight.getPosition(),DRIVE_TRAIN_DEFAULT_SPEED);

        /* Complete color sensor setup
           Note: These initialization methods could not be performed in the respective object constructors */
        //sampleSensor.initialize();

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

      /*  // Blink all relicRobot lights/LEDs x2 to acknowledge initialization
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
*/
        // Report initialization complete
        telemetry.addLine("Initialization Complete");
        telemetry.addLine("Hold for Start");
        telemetry.update();


        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        initVuforia();

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start tracking");
        telemetry.update();


        // WAIT FOR THE GAME TO START (driver presses PLAY)
        waitForStart();

        autoTimer.reset();  // Reset/restart the autotimer


        // GAME STARTED - BEGIN AUTONOMOUS OPERATIONS


        if (opModeIsActive()) {
            /** Activate Tensor Flow Object Detection. */
            if (tfod != null) {
                tfod.activate();
            }

            while (autoTimer.seconds() < 25) {
                if (tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                        telemetry.addData("# Object Detected", updatedRecognitions.size());
                        if (updatedRecognitions.size() == 2) {
                            int goldMineralX = -1;
                            int silverMineral1X = -1;
                            for (Recognition recognition : updatedRecognitions) {
                                if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                                    goldMineralX = (int) recognition.getLeft();
                                } else  {
                                    silverMineral1X = (int) recognition.getLeft();
                                }
                            }
                            if (goldMineralX != -1 && silverMineral1X != -1) {
                                if (goldMineralX < silverMineral1X) {
                                    telemetry.addData("Gold Mineral Position", "Center");
                                    goldCenter = true;
                                } else if (goldMineralX > silverMineral1X) {
                                    telemetry.addData("Gold Mineral Position", "Right");
                                    goldRight = true;
                                }
                            }
                            else {
                                goldLeft = true;
                                telemetry.addData("Gold Mineral Position", "Left");
                            }
                        }
                        telemetry.update();
                    }
                }
            }
        }

        if (tfod != null) {
            tfod.shutdown();
        }


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

    void motorTelemetry (DeviceTargetMotor motor) {
        telemetry.addLine();
        telemetry.addLine(motor.name);
        telemetry.addData(" Position in eng. units", "%.2f", motor.getPosition());
        telemetry.addData(" Position in Counts", motor.targetMotor.getCurrentPosition());
    }


    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the Tensor Flow Object Detection engine.
    }

    /**
     * Initialize the Tensor Flow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
    }
}