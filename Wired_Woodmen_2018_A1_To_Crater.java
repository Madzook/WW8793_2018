package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;



@Autonomous(name="Wired_Woodmen_2018_A1_To_Crater", group="Wired Woodman")

public class Wired_Woodmen_2018_A1_To_Crater extends AutoLinearAbstract_2018 {

    //Created for position that is not in front of crater


    public void runOpMode() {

        super.runOpMode();


        latchArmGripper.goToPositionNow(LATCH_ARM_GRIPPER_CLOSED_POS);
        latchArm.goToAbsoluteDegrees(90, .1);


        generalTimer.reset();
        //extends latchArm
        //opens up the claw

        latchArm.goToRelativeDistance(-16500, 1);
        backExtension.goToPositionNow(BACK_EXTEND_POS);
        generalTimer.reset();
        while (!latchArm.isMoveDone(100)&& generalTimer.seconds() > 3.0) {
            telemetry.addLine("Wait - Latch Arm landing the robot");
            motorTelemetry(latchArm);
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

        driveTrain.goRoundToTarget(0, 33, DRIVE_TRAIN_DEFAULT_SPEED);
        while (!driveTrain.isMoveDone(MAX_DRIVE_TRAIN_POSITION_ERROR_INCHES)) {
            telemetry.addLine("Wait - Drive train move left towards crater");
            driveTrainTelemetry();
            telemetry.update();
            if (autoTimer.seconds() > 28) {
                driveTrain.stop();
                break;
            }
        }
        driveTrain.goStraightToTarget(55, DRIVE_TRAIN_DEFAULT_SPEED);
        while (!driveTrain.isMoveDone(MAX_DRIVE_TRAIN_POSITION_ERROR_INCHES)) {
            telemetry.addLine("Wait - Drive train move straight towards crater");
            driveTrainTelemetry();
            telemetry.update();
            if (autoTimer.seconds() > 28) {
                driveTrain.stop();
                break;
            }
        }

    }
}