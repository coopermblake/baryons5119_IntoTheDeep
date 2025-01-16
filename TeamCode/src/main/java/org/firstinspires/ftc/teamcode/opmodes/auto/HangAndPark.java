package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.core.Robot;

@Autonomous(name = "Hang and park", group = "Competition Opmodes")
public class HangAndPark extends LinearOpMode {
    public void runOpMode() throws InterruptedException {
        // TODO: huge hack, please replace this later
        Robot robot = new Robot(hardwareMap, gamepad1, gamepad2);
        robot.gripper.setPosition(0.81);
        waitForStart();
        int lowPosition = robot.slideRot.getCurrentPosition();
        int minPosition = robot.slideExt.getCurrentPosition();
        robot.slideRot.setTargetPosition(lowPosition - 2800);
        robot.slideExt.setTargetPosition(minPosition + 1200);
        robot.slideRot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.slideRot.setPower(1);
        android.os.SystemClock.sleep(500);
        robot.slideExt.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.slideExt.setPower(0.2);

        robot.backLeft.setPower(0.3);
        robot.backRight.setPower(0.3);
        robot.frontLeft.setPower(0.3);
        robot.frontRight.setPower(0.3);
        android.os.SystemClock.sleep(5000);
        robot.backLeft.setPower(0);
        robot.backRight.setPower(0);
        robot.frontLeft.setPower(0);
        robot.frontRight.setPower(0);
        android.os.SystemClock.sleep(1000);
        robot.slideExt.setTargetPosition(minPosition + 200);
        robot.slideRot.setTargetPosition(lowPosition - 2700);
        android.os.SystemClock.sleep(2000);
        robot.gripper.setPosition(0.47);
        android.os.SystemClock.sleep(1000);

        robot.slideRot.setTargetPosition(lowPosition);
        robot.slideExt.setTargetPosition(minPosition);

        robot.backLeft.setPower(-0.25);
        robot.backRight.setPower(-0.25);
        robot.frontLeft.setPower(-0.25);
        robot.frontRight.setPower(-0.25);
        android.os.SystemClock.sleep(3000);

        robot.backLeft.setPower(0.15);
        robot.backRight.setPower(0.15);
        robot.frontLeft.setPower(0.15);
        robot.frontRight.setPower(0.15);

        android.os.SystemClock.sleep(1000);

        robot.backLeft.setPower(0);
        robot.backRight.setPower(0);
        robot.frontLeft.setPower(0);
        robot.frontRight.setPower(0);

        android.os.SystemClock.sleep(500);

        robot.backLeft.setPower(-0.25);
        robot.backRight.setPower(0.25);
        robot.frontLeft.setPower(0.25);
        robot.frontRight.setPower(-0.25);
        android.os.SystemClock.sleep(5000);
    }
}
