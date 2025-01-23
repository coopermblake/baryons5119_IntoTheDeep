package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.core.Robot;

@Autonomous(name = "Hang and park encoder", group = "Competition Opmodes")
public class HangAndParkEncoder extends LinearOpMode {
    public void runOpMode() throws InterruptedException {
        // TODO: huge hack, please replace this later

        //initialize stuff
        Robot robot = new Robot(hardwareMap, gamepad1, gamepad2);
        robot.initAutoDrive(telemetry);

        //close gripper to hold specimen
        robot.gripper.setPosition(0.81);
        waitForStart();

        //move slide to hanging position
        int lowPosition = robot.slideRot.getCurrentPosition();
        int minPosition = robot.slideExt.getCurrentPosition();
        robot.slideRot.setTargetPosition(lowPosition - 2800);
        robot.slideExt.setTargetPosition(minPosition + 1200);
        robot.slideRot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.slideRot.setPower(1);
        robot.slideExt.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.slideExt.setPower(0.2);

        sleep(1000);

        //drive to middle
        robot.autoDrive.driveStraightEncoder(0.2, 28, 0.0);
        sleep(500);

        //retract and move arm down
        robot.slideExt.setTargetPosition(minPosition);
        robot.slideRot.setTargetPosition(lowPosition - 2000);
        sleep(1000);

        //release specimen to hang it
        robot.gripper.setPosition(0.47);

        //go right towards floor pieces
        robot.autoDrive.driveStrafeEncoder(0.4, 32, 0.0);

        //move to other side of floor pieces
        robot.autoDrive.driveStraightEncoder(0.2, 28, 0.0);

        //line up robot with piece
        robot.autoDrive.driveStrafeEncoder(0.4, 10, 0.0);


        telemetry.addLine("driving complete");
        telemetry.update();
        sleep(1000);

        //turn around to push with front bumper
        robot.autoDrive.turnToHeadingEncoder(0.3, 180);
        robot.autoDrive.holdHeading(0.2, 180, 1.0);

        //push piece towards human
        robot.autoDrive.driveStraightEncoder(0.4, 39, 180.0);

        //slowly drive forward

        //grab piece

        //reverse

        //turn around

        //strafe left towards bars

        //drive forward towards bars

        //repeat hanging specimen, pushing new piece, picking up specimen, as many times as needed



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
