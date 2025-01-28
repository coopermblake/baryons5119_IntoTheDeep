package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.core.Robot;

@Autonomous(name = "EncoderHang3", group = "Competition Opmodes")
public class Hang3AndParkEncoder extends LinearOpMode {
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap, gamepad1, gamepad2);
        robot.initAutoDrive(telemetry);
        robot.gripper.setPosition(0.81);
        waitForStart();
        int lowPosition = robot.slideRot.getCurrentPosition();
        int minPosition = robot.slideExt.getCurrentPosition();

        //initial hang sequence
        robot.slideRot.setTargetPosition(lowPosition - 2800);
        robot.slideExt.setTargetPosition(minPosition + 1400); //1300
        robot.slideRot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.slideRot.setPower(1.0);
        robot.slideExt.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.slideExt.setPower(1.0);

        //driving forward for hang
        sleep(500); //essential sleep
        robot.autoDrive.driveStraight(0.4, 26, 0.0); //28

        //hang
        robot.slideExt.setTargetPosition(minPosition + 650);
        robot.slideRot.setTargetPosition(lowPosition - 2100);
        sleep(200);

        //release
        robot.gripper.setPosition(0.47);
        robot.slideExt.setTargetPosition(minPosition);

        //strafe away from submersible
        robot.autoDrive.driveStrafe(0.6, 29, 0.0); //may need adjusting
        robot.autoDrive.holdHeading(0.6,0,0.2);

        //raise arm
        robot.slideRot.setTargetPosition(lowPosition - 1100);
        robot.slideRot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.slideRot.setPower(1);

        //drive over the first sample on floor
        robot.autoDrive.driveDiagonal(0.6, 12, 26,1); //13x may need adjusting

        //turn around
        robot.autoDrive.turnToHeading(0.6, 180);
        robot.autoDrive.holdHeading(0.6, 180, 0.4);

        //push sample forwards
        robot.autoDrive.driveStraight(0.6, 47, 180.0);

        //pick up second specimen
        //robot.slideRot.setTargetPosition(lowPosition - 1100); old pivot
        //robot.slideRot.setTargetPosition(lowPosition - 1200);
        robot.slideRot.setTargetPosition(lowPosition - 1150);
        //robot.slideExt.setTargetPosition(minPosition + 1000); old extension
        robot.slideExt.setTargetPosition(minPosition + 700);

        robot.slideRot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.slideExt.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.slideExt.setPower(1);
        while((robot.slideExt.isBusy() || robot.slideRot.isBusy()) && opModeIsActive()) {}
        robot.gripper.setPosition(0.81);
        for (int i = 0; i < 12; i++) {
            sleep(100);
            if(!opModeIsActive()) {
                break;
            }
        }

        //put pivot to hang position, drive back a little
        robot.slideRot.setTargetPosition(lowPosition - 3000); //2900 before
        robot.slideRot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.slideRot.setPower(1);

        sleep(100);

        //rotate
        robot.autoDrive.turnToHeading(0.6,0);
        robot.autoDrive.holdHeading(0.6,0,0.4);

        //drive over to the hang bar
        robot.slideExt.setTargetPosition(minPosition + 1400); //1300
        robot.slideExt.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.slideExt.setPower(1);
        robot.autoDrive.driveDiagonal(0.6,-44,18,7); //adjust this as needed as well

        robot.autoDrive.driveStraight(0.5,5,0); //this will need to be adjusted as needed

        //second hang
        robot.slideExt.setTargetPosition(minPosition + 650);
        robot.slideRot.setTargetPosition(lowPosition - 2000);
        //while (robot.slideRot.isBusy()) {}
        sleep(200);

        //second hang release
        robot.gripper.setPosition(0.47);
        robot.slideExt.setTargetPosition(minPosition);

        //drive over to the next specimen rotate and drive forward
        robot.gripper.setPosition(0.47);
        //robot.slideRot.setTargetPosition(lowPosition - 1100); first one
        //robot.slideRot.setTargetPosition(lowPosition - 1200); second one
        robot.slideRot.setTargetPosition(lowPosition - 1150);
        robot.slideRot.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.autoDrive.driveDiagonal(0.6,42,-17,7);
        robot.autoDrive.turnToHeading(0.6,180);
        robot.autoDrive.holdHeading(0.6,180,0.4);
        robot.autoDrive.driveStraight(0.5, 5,0);

        //pick up the next specimen
        //robot.slideExt.setTargetPosition(minPosition + 1000);
        robot.slideExt.setTargetPosition(minPosition + 700);
        robot.slideExt.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.slideExt.setPower(1);
        while((robot.slideExt.isBusy() || robot.slideRot.isBusy()) && opModeIsActive()) {}
        robot.gripper.setPosition(0.81);
        for (int i = 0; i < 6; i++) {
            sleep(100);
            if(!opModeIsActive()) {
                break;
            }
        }

        //put pivot to hang position, drive back a little
        robot.slideRot.setTargetPosition(lowPosition - 3000);
        robot.slideRot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.slideRot.setPower(1);
        robot.slideExt.setTargetPosition(minPosition);

        //rotate
        robot.autoDrive.turnToHeading(0.6,0);
        robot.autoDrive.holdHeading(0.6,0,0.4);

        //drive over to the hang bar
        robot.slideExt.setTargetPosition(minPosition + 1400);
        robot.slideExt.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.autoDrive.driveDiagonal(0.8,-47,20,7); //adjust as needed
        robot.slideExt.setPower(1);
        robot.autoDrive.driveStraight(0.4,5.0,0);

        //second hang
        robot.slideExt.setTargetPosition(minPosition + 650);
        robot.slideRot.setTargetPosition(lowPosition - 2000);
        while (robot.slideRot.isBusy()) {}

        //release
        robot.gripper.setPosition(0.47);
        robot.slideExt.setTargetPosition(minPosition);
        robot.slideRot.setTargetPosition(lowPosition + 650);
        sleep(1000);
    }
}