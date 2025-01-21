package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.core.Robot;

@Autonomous(name = "10Hang and park encoder", group = "Competition Opmodes")
public class HangAndParkEncoder extends LinearOpMode {
    public void runOpMode() throws InterruptedException {
        // TODO: huge hack, please replace this later
        Robot robot = new Robot(hardwareMap, gamepad1, gamepad2);
        robot.initAutoDrive(telemetry);
        robot.gripper.setPosition(0.81);
        waitForStart();
        int lowPosition = robot.slideRot.getCurrentPosition();
        int minPosition = robot.slideExt.getCurrentPosition();
        robot.slideRot.setTargetPosition(lowPosition - 2800);
        robot.slideExt.setTargetPosition(minPosition + 1200);
        robot.slideRot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.slideRot.setPower(1);
        robot.slideExt.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.slideExt.setPower(0.2);

        sleep(1000);
        robot.autoDrive.driveStraightEncoder(0.2, 28, 0.0);
        sleep(500);

        robot.slideExt.setTargetPosition(minPosition);
        robot.slideRot.setTargetPosition(lowPosition - 2000);
        sleep(1000);

        robot.gripper.setPosition(0.47);

        robot.autoDrive.driveStrafeEncoder(0.4, 32, 0.0);

        robot.autoDrive.driveStraightEncoder(0.2, 28, 0.0);
//
        robot.autoDrive.driveStrafeEncoder(0.4, 10, 0.0);


        telemetry.addLine("driving complete");
        telemetry.update();
        sleep(1000);

        robot.autoDrive.turnToHeadingEncoder(0.3, 180);
        robot.autoDrive.holdHeading(0.2, 180, 1.0);

        robot.autoDrive.driveStraightEncoder(0.4, 39, 180.0);

//        robot.slideRot.setTargetPosition(lowPosition);
//        robot.slideExt.setTargetPosition(minPosition);
//
//        robot.backLeft.setPower(-0.25);
//        robot.backRight.setPower(-0.25);
//        robot.frontLeft.setPower(-0.25);
//        robot.frontRight.setPower(-0.25);
//        sleep(3000);
//
//        robot.backLeft.setPower(0.15);
//        robot.backRight.setPower(0.15);
//        robot.frontLeft.setPower(0.15);
//        robot.frontRight.setPower(0.15);
//
//        sleep(1000);
//
//        robot.backLeft.setPower(0);
//        robot.backRight.setPower(0);
//        robot.frontLeft.setPower(0);
//        robot.frontRight.setPower(0);
//
//        sleep(500);
//
//        robot.backLeft.setPower(-0.25);
//        robot.backRight.setPower(0.25);
//        robot.frontLeft.setPower(0.25);
//        robot.frontRight.setPower(-0.25);
//        sleep(5000);
    }
}
