package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Auto_Drop_Marker_Go_Crater_From_Crater", group="Wired Woodman")
public class Auto_Drop_Marker_Go_Crater_From_Crater extends AutoLinearAbstract_2018 {


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

        //Moving forward
        driveTrain.goRoundToTarget(18,15, DRIVE_TRAIN_DEFAULT_SPEED);
        mineralCollector.goToAbsoluteDistance(MINERAL_COLLECTOR_UP_POS, .1);
        frontExtension.goToPositionNow(FRONT_EXTEND_POS);
        backExtension.goToPositionNow(BACK_EXTEND_POS);
        while (!driveTrain.isMoveDone(MAX_DRIVE_TRAIN_POSITION_ERROR_INCHES )) {
            telemetry.addLine("Wait - Drive train move straight towards crater");
            driveTrainTelemetry();
            motorTelemetry(mineralCollector);
            motorTelemetry(mineralMove);
            telemetry.update();
            if (autoTimer.seconds() > 28) {
                driveTrain.stop();
                break;
            }
        }

        //Turning towards crater
        driveTrain.turnCcwToTarget(13, DRIVE_TRAIN_DEFAULT_SPEED);
        while(!driveTrain.isMoveDone(MAX_DRIVE_TRAIN_POSITION_ERROR_INCHES )) {
            telemetry.addLine("Wait - Robot Turning CounterClockWise towards wall");
            driveTrainTelemetry();
            telemetry.update();
            if (autoTimer.seconds() > 28) {
                driveTrain.stop();
                break;
            }
        }

            //Go To Crater
            driveTrain.goRoundToTarget(35, 35, DRIVE_TRAIN_DEFAULT_SPEED);
            while (!driveTrain.isMoveDone(MAX_DRIVE_TRAIN_POSITION_ERROR_INCHES )) {
                telemetry.addLine("Wait - Robot moving towards wall");
                driveTrainTelemetry();
                telemetry.update();
                if (autoTimer.seconds() > 28) {
                    driveTrain.stop();
                    break;
                }

            }

            driveTrain.turnCcwToTarget(5, DRIVE_TRAIN_DEFAULT_SPEED);
            while(!driveTrain.isMoveDone(MAX_DRIVE_TRAIN_POSITION_ERROR_INCHES )) {
                telemetry.addLine("wait - Robot Turning towards team marker area");
                driveTrainTelemetry();
                telemetry.update();
                if (autoTimer.seconds() > 28) {
                    driveTrain.stop();
                    break;
                }

            }

            driveTrain.goRoundToTarget(44,44, DRIVE_TRAIN_DEFAULT_SPEED);
            mineral_moving = false;
            while (!driveTrain.isMoveDone(MAX_DRIVE_TRAIN_POSITION_ERROR_INCHES) || !mineral_moving) {
                if (!mineral_moving)
                    if (mineralCollector.getPosition() > MINERAL_COLLECTOR_SAFE_LIMIT) {
                        mineralMove.goToAbsoluteDistance(MINERAL_MOVE_UP_POS, .1);
                        mineral_moving = true;  }
                telemetry.addLine("Wait - Moving Again towards team marker area");
                driveTrainTelemetry();
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

        mineralCollector.goToAbsoluteDistance(MINERAL_COLLECTOR_DOWN_POS, .35);

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


        driveTrain.goStraightToTarget(-62, DRIVE_TRAIN_DEFAULT_SPEED);
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
