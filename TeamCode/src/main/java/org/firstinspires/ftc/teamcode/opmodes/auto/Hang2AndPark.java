package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.core.Robot;

@Autonomous(name = "Hang 2 And Park", group = "Competition Opmodes")
public class Hang2AndPark extends LinearOpMode {
    public void runOpMode() throws InterruptedException {
        // TODO: huge hack, please replace this later
        //this does not do parking

        Robot robot = new Robot(hardwareMap, gamepad1, gamepad2);
        robot.gripper.setPosition(0.81);
        waitForStart();
        robot.viperSlide.resetEncoder();
        int lowPosition = robot.slideRot.getCurrentPosition();
        int minPosition = robot.slideExt.getCurrentPosition();
        robot.slideRot.setTargetPosition(lowPosition - 2900);
        robot.slideExt.setTargetPosition(minPosition + 1200);
        robot.slideRot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.slideRot.setPower(1);
        robot.slideExt.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.slideExt.setPower(0.2);
        android.os.SystemClock.sleep(500);

        //go forward
        robot.backLeft.setPower(0.3);
        robot.backRight.setPower(0.3);
        robot.frontLeft.setPower(0.3);
        robot.frontRight.setPower(0.3);

        android.os.SystemClock.sleep(2300);

        //stop
        robot.backLeft.setPower(0);
        robot.backRight.setPower(0);
        robot.frontLeft.setPower(0);
        robot.frontRight.setPower(0);

        android.os.SystemClock.sleep(1000);

        //drop arm
        robot.slideExt.setTargetPosition(minPosition + 200);
        robot.slideRot.setTargetPosition(lowPosition - 2700);

        android.os.SystemClock.sleep(2000);

        //release specimen
        robot.gripper.setPosition(0.47);

        android.os.SystemClock.sleep(1000);

        //reset arm
        robot.slideRot.setTargetPosition(lowPosition);
        robot.slideExt.setTargetPosition(minPosition);

        //reverse
        robot.backLeft.setPower(-0.45);
        robot.backRight.setPower(-0.45);
        robot.frontLeft.setPower(-0.45);
        robot.frontRight.setPower(-0.45);
        android.os.SystemClock.sleep(1200);

        //go forward a little
        robot.backLeft.setPower(0.5);
        robot.backRight.setPower(0.5);
        robot.frontLeft.setPower(0.5);
        robot.frontRight.setPower(0.5);

        android.os.SystemClock.sleep(400);

        //stop
        robot.backLeft.setPower(0);
        robot.backRight.setPower(0);
        robot.frontLeft.setPower(0);
        robot.frontRight.setPower(0);

        android.os.SystemClock.sleep(500);

        //go right (from the view of drivers)
        robot.backLeft.setPower(-0.65);
        robot.backRight.setPower(0.65);
        robot.frontLeft.setPower(0.65);
        robot.frontRight.setPower(-0.65);

        android.os.SystemClock.sleep(1450);

        //stop
        robot.backLeft.setPower(0);
        robot.backRight.setPower(0);
        robot.frontLeft.setPower(0);
        robot.frontRight.setPower(0);
        android.os.SystemClock.sleep(500);

        //rotate 180
        robot.backLeft.setPower(-0.45);
        robot.backRight.setPower(0.45);
        robot.frontLeft.setPower(-0.45);
        robot.frontRight.setPower(0.45);

        android.os.SystemClock.sleep(1200);

        //stop
        robot.backLeft.setPower(0);
        robot.backRight.setPower(0);
        robot.frontLeft.setPower(0);
        robot.frontRight.setPower(0);

        //pivot up a little
        robot.slideRot.setTargetPosition(lowPosition - 1000);
        robot.slideRot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.slideRot.setPower(1);

        //forward a little (this is to slam the robot and make it straight)
        robot.backLeft.setPower(0.5);
        robot.backRight.setPower(0.5);
        robot.frontLeft.setPower(0.5);
        robot.frontRight.setPower(0.5);

        android.os.SystemClock.sleep(800);

        //reverse a little and set gripper position
        robot.backLeft.setPower(-0.25);
        robot.backRight.setPower(-0.25);
        robot.frontLeft.setPower(-0.25);
        robot.frontRight.setPower(-0.25);
        android.os.SystemClock.sleep(400);

        robot.gripper.setPosition(0.47);
        android.os.SystemClock.sleep(300);

        //stop and wait for human player
        robot.backLeft.setPower(0);
        robot.backRight.setPower(0);
        robot.frontLeft.setPower(0);
        robot.frontRight.setPower(0);
        android.os.SystemClock.sleep(1000);

        //forward again
        robot.backLeft.setPower(0.15);
        robot.backRight.setPower(0.15);
        robot.frontLeft.setPower(0.15);
        robot.frontRight.setPower(0.15);

        android.os.SystemClock.sleep(1800);

        //stop
        robot.backLeft.setPower(0);
        robot.backRight.setPower(0);
        robot.frontLeft.setPower(0);
        robot.frontRight.setPower(0);
        android.os.SystemClock.sleep(300);

        //extend arm
        robot.viperSlide.slideExt.setTargetPosition(minPosition+400);
        robot.slideExt.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.slideExt.setPower(0.4);
        android.os.SystemClock.sleep(300);

        //close gripper
        robot.gripper.setPosition(0.81);
        android.os.SystemClock.sleep(500);

        //stop
        robot.backLeft.setPower(0);
        robot.backRight.setPower(0);
        robot.frontLeft.setPower(0);
        robot.frontRight.setPower(0);
        android.os.SystemClock.sleep(300);

        robot.slideRot.setTargetPosition(lowPosition - 1600);
        robot.slideRot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.slideRot.setPower(1);
        android.os.SystemClock.sleep(300);

        //backward a little
        robot.backLeft.setPower(-0.45);
        robot.backRight.setPower(-0.45);
        robot.frontLeft.setPower(-0.45);
        robot.frontRight.setPower(-0.45);
        android.os.SystemClock.sleep(300);

        //stop
        robot.backLeft.setPower(0);
        robot.backRight.setPower(0);
        robot.frontLeft.setPower(0);
        robot.frontRight.setPower(0);
        android.os.SystemClock.sleep(100);

        //rotate 180
        robot.backLeft.setPower(0.48);
        robot.backRight.setPower(-0.48);
        robot.frontLeft.setPower(0.48);
        robot.frontRight.setPower(-0.48);

        android.os.SystemClock.sleep(950);


        robot.backLeft.setPower(0);
        robot.backRight.setPower(0);
        robot.frontLeft.setPower(0);
        robot.frontRight.setPower(0);
        android.os.SystemClock.sleep(100);


        //go left (from the view of drivers)
        robot.backLeft.setPower(0.65);
        robot.backRight.setPower(-0.65);
        robot.frontLeft.setPower(-0.65);
        robot.frontRight.setPower(0.65);

        android.os.SystemClock.sleep(1400);

        // slam backward in order to orientate robot straight
        robot.backLeft.setPower(-0.35);
        robot.backRight.setPower(-0.35);
        robot.frontLeft.setPower(-0.35);
        robot.frontRight.setPower(-0.35);
        android.os.SystemClock.sleep(700);

        //stop
        robot.backLeft.setPower(0);
        robot.backRight.setPower(0);
        robot.frontLeft.setPower(0);
        robot.frontRight.setPower(0);
        android.os.SystemClock.sleep(300);

        //repeat the beginning arm stuff
        robot.slideRot.setTargetPosition(lowPosition - 2800);
        //robot.slideRot.setTargetPosition(lowPosition - 2700);
        robot.slideExt.setTargetPosition(minPosition + 1400);
        robot.slideRot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.slideRot.setPower(1);
        robot.slideExt.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.slideExt.setPower(0.2);
        android.os.SystemClock.sleep(500);

        //go forward
        robot.backLeft.setPower(0.35);
        robot.backRight.setPower(0.35);
        robot.frontLeft.setPower(0.35);
        robot.frontRight.setPower(0.35);

        android.os.SystemClock.sleep(2300);

        //slow
        robot.backLeft.setPower(0.1);
        robot.backRight.setPower(0.1);
        robot.frontLeft.setPower(0.1);
        robot.frontRight.setPower(0.1);


        //drop arm while driving forward slow
        robot.slideExt.setTargetPosition(minPosition + 200);
        robot.slideRot.setTargetPosition(lowPosition - 2700);

        android.os.SystemClock.sleep(2700);


        robot.backLeft.setPower(0.);
        robot.backRight.setPower(0.1);
        robot.frontLeft.setPower(0);
        robot.frontRight.setPower(0.1);
        android.os.SystemClock.sleep(100);

        //wiggle protocol
        robot.backLeft.setPower(0.1);
        robot.backRight.setPower(0);
        robot.frontLeft.setPower(0.1);
        robot.frontRight.setPower(0);
        android.os.SystemClock.sleep(650);
        robot.backLeft.setPower(0);
        robot.backRight.setPower(0.1);
        robot.frontLeft.setPower(0);
        robot.frontRight.setPower(0.1);
        android.os.SystemClock.sleep(650);
        robot.backLeft.setPower(0);
        robot.backRight.setPower(0.1);
        robot.frontLeft.setPower(0);
        robot.frontRight.setPower(0.1);
        android.os.SystemClock.sleep(650);



        //release specimen
        robot.gripper.setPosition(0.47);

        android.os.SystemClock.sleep(200);

        //reset arm
        robot.slideRot.setTargetPosition(lowPosition);
        robot.slideExt.setTargetPosition(minPosition);
    }
}
