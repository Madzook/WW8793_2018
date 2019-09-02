package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Wired_Woodmen_2018_Lower_And_Open", group="Wired Woodman")

public class Wired_Woodmen_2018_Lower_And_Open extends AutoLinearAbstract_2018 {



    public void runOpMode() {

        super.runOpMode();


        latchArmGripper.goToPositionNow(LATCH_ARM_GRIPPER_CLOSED_POS);

        generalTimer.reset();
        //extends latchArm
        //opens up the claw

        latchArm.goToRelativeDistance(-16500, 1);
        backExtension.goToPositionNow(BACK_RETRACT_POS);
        while (!latchArm.isMoveDone(100)) {
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


    }
}