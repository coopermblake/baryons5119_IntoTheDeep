package org.firstinspires.ftc.teamcode.opmodes.driverControl;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.core.Robot;

@TeleOp(name="MotorTest",group="TEST")
public class testMode extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        //boolean heldRight = false;
        Robot robot = new Robot(hardwareMap, gamepad1, gamepad2);
        waitForStart();

        int[] encoders = new int[4];
        encoders[0] = robot.backLeft.getCurrentPosition();
        encoders[1] = robot.backRight.getCurrentPosition();
        encoders[2] = robot.frontLeft.getCurrentPosition();
        encoders[3] = robot.frontRight.getCurrentPosition();

        robot.backLeft.setPower(0.5);
        robot.backRight.setPower(0.5);
        robot.frontLeft.setPower(0.5);
        robot.frontRight.setPower(0.5);

        android.os.SystemClock.sleep(125);

        robot.backLeft.setPower(0);
        robot.backRight.setPower(0);
        robot.frontLeft.setPower(0);
        robot.frontRight.setPower(0);

        if (robot.backLeft.getCurrentPosition() > encoders[0] + 20) {
            telemetry.addLine("backLeft encoder test passed");
        } else if (robot.backLeft.getCurrentPosition() < encoders[0] - 20) {
            telemetry.addLine("backLeft encoder is reversed");
        } else {
            telemetry.addLine("backLeft encoder is not ticking");
        }

        if (robot.backRight.getCurrentPosition() > encoders[1] + 20) {
            telemetry.addLine("backRight encoder test passed");
        } else if (robot.backRight.getCurrentPosition() < encoders[1] - 20) {
            telemetry.addLine("backRight encoder is reversed");
        } else {
            telemetry.addLine("backRight encoder is not ticking");
        }

        if (robot.frontLeft.getCurrentPosition() > encoders[2] + 20) {
            telemetry.addLine("frontLeft encoder test passed");
        } else if (robot.frontLeft.getCurrentPosition() < encoders[2] - 20) {
            telemetry.addLine("frontLeft encoder is reversed");
        } else {
            telemetry.addLine("frontLeft encoder is not ticking");
        }

        if (robot.frontRight.getCurrentPosition() > encoders[3] + 20) {
            telemetry.addLine("frontRight encoder test passed");
        } else if (robot.frontRight.getCurrentPosition() < encoders[3] - 20) {
            telemetry.addLine("frontRight encoder is reversed");
        } else {
            telemetry.addLine("frontRight encoder is not ticking");
        }


        telemetry.addLine("Press A to continue.");
        telemetry.update();
        while (!gamepad1.a) {

        }
        robot.slideRot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        while(opModeIsActive()) {
            telemetry.addData("backLeft encoder value:", robot.backLeft.getCurrentPosition());
            telemetry.addData("backRight encoder value:", robot.backRight.getCurrentPosition());
            telemetry.addData("frontLeft encoder value:", robot.frontLeft.getCurrentPosition());
            telemetry.addData("frontRight encoder value:", robot.frontRight.getCurrentPosition());
            telemetry.addData("slideExt encoder value:", robot.slideExt.getCurrentPosition());
            telemetry.addData("slideRot encoder value:", robot.slideRot.getCurrentPosition());
            telemetry.addData("gripper encoder value:" , robot.gripper.getPosition());
            telemetry.addData("imu yaw", robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
            telemetry.update();

            if (robot.gamepad1.a) {
                robot.backLeft.setPower(0.5);
            } else {
                robot.backLeft.setPower(0);
            }
            if (robot.gamepad1.b) {
                robot.backRight.setPower(0.5);
            } else {
                robot.backRight.setPower(0);
            }
            if (robot.gamepad1.x) {
                robot.frontLeft.setPower(0.5);
            } else {
                robot.frontLeft.setPower(0);
            }
            if (robot.gamepad1.y) {
                robot.frontRight.setPower(0.5);
            } else {
                robot.frontRight.setPower(0);
            }
            if (robot.gamepad1.dpad_up) {
                robot.slideRot.setPower(0.5);
            } else if (robot.gamepad1.dpad_down) {
                robot.slideRot.setPower(-0.5);
            } else {
                robot.slideRot.setPower(0);
            }
            if(robot.gamepad1.dpad_right) {
                robot.slideExt.setPower(0.5);
            } else if(robot.gamepad1.dpad_left) {
                robot.slideExt.setPower(-0.5);
            } else {
                robot.slideExt.setPower(0);
            }
            if (robot.gamepad1.right_trigger > 0.05) {
                robot.gripper.setPosition(0.81);
            } else if (robot.gamepad1.left_trigger > 0.05) {
                robot.gripper.setPosition(0.47);
            }
        }
    }
}
