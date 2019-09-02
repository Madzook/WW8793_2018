package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Auto_Lower_Go_Crater", group="Wired Woodman")

public class Auto_Lower_Go_Crater extends AutoLinearAbstract_2018 {



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
        latchArmGripper.goToPosition(LATCH_ARM_GRIPPER_OPEN_POS, .01);

        //Go to crater
        driveTrain.goStraightToTarget(32, DRIVE_TRAIN_DEFAULT_SPEED);
        frontExtension.goToPositionNow(FRONT_EXTEND_POS);
        backExtension.goToPositionNow(BACK_EXTEND_POS);
        while (!driveTrain.isMoveDone(MAX_DRIVE_TRAIN_POSITION_ERROR_INCHES)) {
            telemetry.addLine("Wait - Drive train move straight towards crater area");
            driveTrainTelemetry();
            telemetry.update();
            if (autoTimer.seconds() > 28) {
                driveTrain.stop();
                break;
            }
        }



    }
}