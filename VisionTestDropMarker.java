package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Disabled
//@Autonomous(name="VisionTestDropMarker", group="Wired Woodman")
public class VisionTestDropMarker extends AutoLinearAbstractCamera2018 {


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


        if (goldLeft = true) {
            //Go to crater
            driveTrain.goRoundToTarget(20, 30, DRIVE_TRAIN_DEFAULT_SPEED);
            mineralCollector.goToAbsoluteDistance(MINERAL_COLLECTOR_UP_POS, .1);
            mineral_moving = false;
            frontExtension.goToPositionNow(FRONT_EXTEND_POS);
            backExtension.goToPositionNow(BACK_EXTEND_POS);
            while (!driveTrain.isMoveDone(MAX_DRIVE_TRAIN_POSITION_ERROR_INCHES) || !mineral_moving) {
                if (!mineral_moving)
                    if (mineralCollector.getPosition() > MINERAL_COLLECTOR_SAFE_LIMIT) {
                        mineralMove.goToAbsoluteDistance(MINERAL_MOVE_UP_POS, .1);
                        mineral_moving = true;
                    }
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

            driveTrain.goRoundToTarget(20,7,DRIVE_TRAIN_DEFAULT_SPEED);
            while (!driveTrain.isMoveDone(MAX_DRIVE_TRAIN_POSITION_ERROR_INCHES)) {
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
            while (!driveTrain.isMoveDone(MAX_DRIVE_TRAIN_POSITION_ERROR_INCHES)) {
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


        if (goldRight = true) {
            //Go to crater
            driveTrain.goRoundToTarget(30, 20, DRIVE_TRAIN_DEFAULT_SPEED);
            mineralCollector.goToAbsoluteDistance(MINERAL_COLLECTOR_UP_POS, .1);
            mineral_moving = false;
            frontExtension.goToPositionNow(FRONT_EXTEND_POS);
            backExtension.goToPositionNow(BACK_EXTEND_POS);
            while (!driveTrain.isMoveDone(MAX_DRIVE_TRAIN_POSITION_ERROR_INCHES) || !mineral_moving) {
                if (!mineral_moving)
                    if (mineralCollector.getPosition() > MINERAL_COLLECTOR_SAFE_LIMIT) {
                        mineralMove.goToAbsoluteDistance(MINERAL_MOVE_UP_POS, .1);
                        mineral_moving = true;
                    }
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

            driveTrain.goRoundToTarget(10,17,DRIVE_TRAIN_DEFAULT_SPEED);
            while (!driveTrain.isMoveDone(MAX_DRIVE_TRAIN_POSITION_ERROR_INCHES)) {
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
            while (!driveTrain.isMoveDone(MAX_DRIVE_TRAIN_POSITION_ERROR_INCHES)) {
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


        if (goldCenter = true) {
            driveTrain.goRoundToTarget(40, 37, DRIVE_TRAIN_DEFAULT_SPEED);
            mineralCollector.goToAbsoluteDistance(MINERAL_COLLECTOR_UP_POS, .1);
            mineral_moving = false;
            frontExtension.goToPositionNow(FRONT_EXTEND_POS);
            backExtension.goToPositionNow(BACK_EXTEND_POS);
            while (!driveTrain.isMoveDone(MAX_DRIVE_TRAIN_POSITION_ERROR_INCHES) || !mineral_moving) {
                if (!mineral_moving)
                    if (mineralCollector.getPosition() > MINERAL_COLLECTOR_SAFE_LIMIT) {
                        mineralMove.goToAbsoluteDistance(MINERAL_MOVE_UP_POS, .1);
                        mineral_moving = true;
                    }
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
            while (!driveTrain.isMoveDone(MAX_DRIVE_TRAIN_POSITION_ERROR_INCHES)) {
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
}