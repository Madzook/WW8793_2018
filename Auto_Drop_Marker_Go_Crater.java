package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Autonomous(name="Auto_Drop_Marker_Go_Crater", group="Wired Woodman")
public class Auto_Drop_Marker_Go_Crater extends AutoLinearAbstract_2018 {



    public void runOpMode() {

        super.runOpMode();


        latchArmGripper.goToPositionNow(LATCH_ARM_GRIPPER_CLOSED_POS);

        //extends latchArm
        //opens up the claw

        latchArm.goToRelativeDistance(-16500, 1);
        backExtension.goToPositionNow(BACK_RETRACT_POS);
        generalTimer.reset(); // The timer gives the servo time to move
        while (!latchArm.isMoveDone(100)|| generalTimer.seconds() < 3.0) {
            telemetry.addLine("Wait - Latch Arm landing the robot");
            motorTelemetry(latchArm);
            telemetry.update();
            if (autoTimer.seconds() > 28) {
                latchArm.stop();
                break;
            }
        }

        telemetry.addLine("Wait - Open gripper");
        telemetry.update();
        latchArmGripper.goToPosition(LATCH_ARM_GRIPPER_OPEN_POS, .01);

        //Go to crater
        driveTrain.goRoundToTarget(40,37, DRIVE_TRAIN_DEFAULT_SPEED);
        mineralCollector.goToAbsoluteDistance(MINERAL_COLLECTOR_UP_POS, .1);
        mineral_moving = false;
        frontExtension.goToPositionNow(FRONT_EXTEND_POS);
        backExtension.goToPositionNow(BACK_EXTEND_POS);
        while (!driveTrain.isMoveDone(MAX_DRIVE_TRAIN_POSITION_ERROR_INCHES ) || !mineral_moving) {
            if (!mineral_moving)
                if (mineralCollector.getPosition() > MINERAL_COLLECTOR_SAFE_LIMIT) {
                    mineralMove.goToAbsoluteDistance(MINERAL_MOVE_UP_POS, .1);
                    mineral_moving = true;  }
            telemetry.addLine("Wait - Drive train move straight towards team marker area");
            driveTrainTelemetry();
            motorTelemetry(mineralCollector);
            motorTelemetry(mineralMove);
            telemetry.update();
            if (autoTimer.seconds() > 28) {
                driveTrain.stop();
                break;
            }
        }

        while (!mineralMove.isMoveDone(100)) {
            telemetry.addLine("Wait - Mineral collector and move going up 1");
            driveTrainTelemetry();
            motorTelemetry(mineralMove);
            motorTelemetry(mineralCollector);
            telemetry.update();
            if (autoTimer.seconds() > 28) {
                driveTrain.stop();
                break;
            }
        }

        while (!mineralCollector.isMoveDone(100)) {
            telemetry.addLine("Wait - Mineral collector and move going up 2");
            driveTrainTelemetry();
            motorTelemetry(mineralMove);
            motorTelemetry(mineralCollector);
            telemetry.update();
            if (autoTimer.seconds() > 28) {
                driveTrain.stop();
                break;
            }
        }

        mineralCollector.goToAbsoluteDistance(MINERAL_COLLECTOR_DOWN_POS, .2);

        while (!mineralCollector.isMoveDone(100)) {
            telemetry.addLine("Wait - Mineral Collector go to down position");
            driveTrainTelemetry();
            motorTelemetry(mineralMove);
            motorTelemetry(mineralCollector);
            telemetry.update();
            if (autoTimer.seconds() > 28) {
                driveTrain.stop();
                break;
            }
        }

        mineralMove.goToAbsoluteDistance(MINERAL_MOVE_DOWN_POS, .1);

        driveTrain.goStraightToTarget(-20, DRIVE_TRAIN_DEFAULT_SPEED);
        while (!driveTrain.isMoveDone(MAX_DRIVE_TRAIN_POSITION_ERROR_INCHES )) {
            telemetry.addLine("Wait - Drive train go back towards lander");
            driveTrainTelemetry();
            motorTelemetry(mineralCollector);
            motorTelemetry(mineralMove);
            telemetry.update();
            if (autoTimer.seconds() > 28) {
                driveTrain.stop();
                break;
            }
        }

        driveTrain.turnCcwToTarget(8, DRIVE_TRAIN_DEFAULT_SPEED);
        while (!driveTrain.isMoveDone(MAX_DRIVE_TRAIN_POSITION_ERROR_INCHES )) {
            telemetry.addLine("Wait - Drive train turn towards crater");
            driveTrainTelemetry();
            motorTelemetry(mineralCollector);
            motorTelemetry(mineralMove);
            telemetry.update();
            if (autoTimer.seconds() > 28) {
                driveTrain.stop();
                break;
            }
        }


        driveTrain.goStraightToTarget(-53, DRIVE_TRAIN_DEFAULT_SPEED);
        while (!driveTrain.isMoveDone(MAX_DRIVE_TRAIN_POSITION_ERROR_INCHES )) {
            telemetry.addLine("Wait - Drive train go to crater");
            driveTrainTelemetry();
            motorTelemetry(mineralCollector);
            motorTelemetry(mineralMove);
            telemetry.update();
            if (autoTimer.seconds() > 28) {
                driveTrain.stop();
                break;
            }
        }

        while (!mineralMove.isMoveDone(100)) {
            telemetry.addLine("Wait - Mineral move go to down position");
            driveTrainTelemetry();
            motorTelemetry(mineralMove);
            motorTelemetry(mineralCollector);
            telemetry.update();
            if (autoTimer.seconds() > 28) {
                driveTrain.stop();
                break;
            }
        }


    }
}