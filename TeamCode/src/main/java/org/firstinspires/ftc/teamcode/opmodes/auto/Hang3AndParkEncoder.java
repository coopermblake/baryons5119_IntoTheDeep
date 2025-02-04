package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.core.Robot;


@Autonomous(name = "EncoderHang3", group = "Competition Opmodes")
public class Hang3AndParkEncoder extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
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
        robot.autoDrive.driveStraight(0.6, 48, 180.0); //47
        //robot.autoDrive.holdHeading(0.6, 180, 0.4);

        //pick up second specimen
        //robot.slideRot.setTargetPosition(lowPosition - 1100); old pivot
        //robot.slideRot.setTargetPosition(lowPosition - 1200);
        robot.slideRot.setTargetPosition(lowPosition - 1140); //1130
        //robot.slideExt.setTargetPosition(minPosition + 1000); old extension
        robot.slideExt.setTargetPosition(minPosition + 900); //700

        robot.slideRot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.slideExt.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.slideRot.setPower(0.80);
        robot.slideExt.setPower(0.75);

        while((robot.slideExt.isBusy() || robot.slideRot.isBusy()) && opModeIsActive()) {}
        robot.gripper.setPosition(0.81);

        wait(1300);

        //put pivot to hang position, drive back a little
        robot.slideRot.setTargetPosition(lowPosition - 3200); //3000 before
        robot.slideRot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.slideRot.setPower(0.80);

        while(opModeIsActive() && robot.slideRot.getCurrentPosition() > (lowPosition - 1500)) {
            sleep(10);
        }

        //sleep(100);

        //rotate
        robot.autoDrive.turnToHeading(0.6,0);
        robot.autoDrive.holdHeading(0.6,0,0.4);

        //drive over to the hang bar
        robot.slideExt.setTargetPosition(minPosition + 1000); //1200
        robot.slideExt.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.slideExt.setPower(1);
        robot.autoDrive.driveDiagonal(0.6,-44.5,17,5); //18

        robot.autoDrive.driveStraight(0.5,7,0); //3

        //second hang
        robot.slideExt.setTargetPosition(minPosition + 650);
        robot.slideRot.setTargetPosition(lowPosition - 2000);
        robot.slideRot.setPower(1.0);
        //while (robot.slideRot.isBusy()) {}
        robot.autoDrive.driveStraight(0.5,-2,0);

        sleep(200);

        //second hang release
        robot.gripper.setPosition(0.47);
        robot.slideExt.setTargetPosition(minPosition);


        //drive over to the next specimen rotate and drive forward
        robot.gripper.setPosition(0.47);
        sleep(200);
        //robot.slideRot.setTargetPosition(lowPosition - 1100); first one
        //robot.slideRot.setTargetPosition(lowPosition - 1200); second one
        robot.slideRot.setTargetPosition(lowPosition - 1140);
        robot.slideRot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.slideRot.setPower(1);

        for (int i = 0; i < 6; i++) {
            if(!opModeIsActive()) {
                break;
            }
            sleep(100);
        }

        robot.autoDrive.driveDiagonal(0.6,42,-17,5);
        robot.autoDrive.turnToHeading(0.6,180);
        robot.autoDrive.holdHeading(0.6,180,0.4);
        robot.autoDrive.driveStraight(0.5, 5,0);

        robot.autoDrive.holdHeading(0.6, 180, 0.2);

        //pick up the third specimen
        //robot.slideExt.setTargetPosition(minPosition + 1000);
        robot.slideExt.setTargetPosition(minPosition + 700);
        robot.slideExt.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.slideExt.setPower(1);

        while((robot.slideExt.isBusy() || robot.slideRot.isBusy()) && opModeIsActive()) {
            sleep(10);
        }
        robot.gripper.setPosition(0.81);
        wait(700);

        //put pivot to hang position, drive back a little
        robot.slideRot.setTargetPosition(lowPosition - 3000);
        robot.slideRot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.slideRot.setPower(1);
        while(opModeIsActive() && robot.slideRot.getCurrentPosition() > (lowPosition - 1500)) {
            sleep(10);
        }

        robot.slideExt.setTargetPosition(minPosition);

        //rotate
        robot.autoDrive.turnToHeading(0.6,0);
        robot.autoDrive.holdHeading(0.6,0,0.4);

        //drive over to the hang bar
        robot.slideExt.setTargetPosition(minPosition + 1400);
        robot.slideExt.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.autoDrive.driveDiagonal(0.8,-47,20,5); //adjust as needed
        robot.slideExt.setPower(1);
        robot.autoDrive.driveStraight(0.4,5.5,0); //4

        //third hang
        robot.slideExt.setTargetPosition(minPosition + 650);
        robot.slideRot.setTargetPosition(lowPosition - 2000);
        while (robot.slideRot.isBusy()) {}

        //release
        robot.gripper.setPosition(0.47);
        robot.slideExt.setTargetPosition(minPosition);
        //robot.slideRot.setTargetPosition(lowPosition + 650);
        sleep(200);

        robot.autoDrive.driveDiagonal(1,50,-30,5); //adjust as needed
    }

    public void wait(int ms) {
        for (int i = 0; i < ms / 100; i++) {
            if(!opModeIsActive()) {
                break;
            }
            sleep(100);
        }
    }

    public void smartSleep(double seconds) {
        runtime.reset(); //restart/set the timer back to 0
        while (opModeIsActive() && runtime.seconds() < 3.0) {
            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }
    }
}