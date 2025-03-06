package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.core.Robot;

@Disabled
public class BasketAuto extends LinearOpMode {
    @Config
    public static class BasketConstants{
        public static double forward = 12;
        public static double left = 12;
        public static double score1angle = 135;

    }
    public void runOpMode(){
        //close gripper
        Robot robot = new Robot(hardwareMap, gamepad1, gamepad2);
        robot.gripper.setPosition(0.81);
        robot.initAutoDrive(telemetry);
        waitForStart();
        robot.viperSlide.resetEncoder();
        robot.gripper.setPosition(0.81);
        int lowPosition = robot.slideRot.getCurrentPosition();
        int minPosition = robot.slideExt.getCurrentPosition();
        waitForStart();
        //start raising arm
        //go forward
        //turn right
        //turn to bar
        //raise arm
        //drop sample 1
        //rotate back slightly

        //simultaneously:
        //lower arm to horizontal
        //retract to pickup sample 2 length

        //lower arm
        //grab sample 2
        //raise arm

        //simultaneously:
        //extend to basket
        //turn to basket

        //drop sample 2

        //simultaneously:
        //lower arm to horizontal
        //retract to pickup sample 3 length

        //lower arm
        //grab sample 3
        //raise arm

        //simultaneously:
        //extend to basket
        //turn to basket

        //drop sample 3

        //simultaneously:
        //lower arm to horizontal
        //retract to pickup sample 4 length

        //lower arm
        //grab sample 4
        //raise arm

        //simultaneously:
        //extend to basket
        //turn to basket

        //drop sample 4



    }
}
