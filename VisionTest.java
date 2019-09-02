package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="VisionTest", group="Wired Woodman")
public class VisionTest extends AutoLinearAbstractCamera2018 {


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
            driveTrain.goRoundToTarget(20,30, DRIVE_TRAIN_DEFAULT_SPEED);
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
        }

        if (goldRight = true) {
            driveTrain.goRoundToTarget(30,20,DRIVE_TRAIN_DEFAULT_SPEED);
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
        }

        if (goldCenter = true) {
            driveTrain.goStraightToTarget(32,DRIVE_TRAIN_DEFAULT_SPEED);
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
        }




    }
}