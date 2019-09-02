package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

//@Autonomous(name="Wired_Woodmen_2018_A4", group="Wired Woodman")
@Disabled



public class Wired_Woodmen_2018_A4 extends AutoLinearAbstract_2018 {

    public void runOpMode() {

        super.runOpMode();


        latchArmGripper.goToPositionNow(LATCH_ARM_GRIPPER_CLOSED_POS);
        latchArm.goToAbsoluteDegrees(90, .1);


        generalTimer.reset();
        //extends latchArm
        //opens up the claw

        latchArm.goToRelativeDistance(-16500, 1);
        while (!latchArm.isMoveDone(5)) {
            telemetry.addLine("Wait - Latch Arm landing the robot");
            motorTelemetryDegrees(latchArm);
            telemetry.update();
            if (autoTimer.seconds() > 28) {
                latchArm.stop();
                break;
            }
        }

        telemetry.addLine("Wait - Open gripper");
        latchArmGripper.goToPosition(LATCH_ARM_GRIPPER_OPEN_POS, .01);


        driveTrain.goStraightToTarget(24, DRIVE_TRAIN_DEFAULT_SPEED);
        while (!driveTrain.isMoveDone(MAX_DRIVE_TRAIN_POSITION_ERROR_INCHES)) {
            telemetry.addLine("Wait - Drive train move straight towards team marker area");
            driveTrainTelemetry();
            telemetry.update();
            if (autoTimer.seconds() > 28) {
                driveTrain.stop();
                break;
            }
        }

        driveTrain.goRoundToTarget(24, 0, DRIVE_TRAIN_DEFAULT_SPEED);
        while (!driveTrain.isMoveDone(MAX_DRIVE_TRAIN_POSITION_ERROR_INCHES)) {
            telemetry.addLine("Wait - Drive train move left towards team marker area");
            driveTrainTelemetry();
            telemetry.update();
            if (autoTimer.seconds() > 28) {
                driveTrain.stop();
                break;
            }
        }

        driveTrain.goStraightToTarget(48, DRIVE_TRAIN_DEFAULT_SPEED);
        while (!driveTrain.isMoveDone(MAX_DRIVE_TRAIN_POSITION_ERROR_INCHES)) {
            telemetry.addLine("Wait - Drive train move straight towards team marker area");
            driveTrainTelemetry();
            telemetry.update();
            if (autoTimer.seconds() > 28) {
                driveTrain.stop();
                break;
            }
        }

        driveTrain.goRoundToTarget(0, 24, DRIVE_TRAIN_DEFAULT_SPEED);
        while (!driveTrain.isMoveDone(MAX_DRIVE_TRAIN_POSITION_ERROR_INCHES)) {
            telemetry.addLine("Wait - Drive train move right towards team marker area");
            driveTrainTelemetry();
            telemetry.update();
            if (autoTimer.seconds() > 28) {
                driveTrain.stop();
                break;
            }
        }

        driveTrain.goStraightToTarget(48, DRIVE_TRAIN_DEFAULT_SPEED);
        while (!driveTrain.isMoveDone(MAX_DRIVE_TRAIN_POSITION_ERROR_INCHES)) {
            telemetry.addLine("Wait - Drive train move straight towards team marker area");
            driveTrainTelemetry();
            telemetry.update();
            if (autoTimer.seconds() > 28) {
                driveTrain.stop();
                break;
            }
        }

        //
        //
        // we need to drop the sample here
        //
        //

        driveTrain.goRoundToTarget(0, 72, DRIVE_TRAIN_DEFAULT_SPEED);
        while (!driveTrain.isMoveDone(MAX_DRIVE_TRAIN_POSITION_ERROR_INCHES)) {
            telemetry.addLine("Wait - Drive train turn right towards samples");
            driveTrainTelemetry();
            telemetry.update();
            if (autoTimer.seconds() > 28) {
                driveTrain.stop();
                break;
            }
        }

        driveTrain.goStraightToTarget(48, DRIVE_TRAIN_DEFAULT_SPEED);
        while (!driveTrain.isMoveDone(MAX_DRIVE_TRAIN_POSITION_ERROR_INCHES)) {
            telemetry.addLine("Wait - Drive train move towards samples");
            driveTrainTelemetry();
            telemetry.update();
            if (autoTimer.seconds() > 28) {
                driveTrain.stop();
                break;
            }
        }

    }
}
