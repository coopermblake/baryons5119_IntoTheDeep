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
        int encoderStart;

        encoderStart = robot.backLeft.getCurrentPosition();
        robot.backLeft.setPower(0.5);
        android.os.SystemClock.sleep(125);
        if (robot.backLeft.getCurrentPosition() > encoderStart + 20) {
            telemetry.addLine("backLeft encoder test passed");
        } else {
            telemetry.addLine("backLeft encoder test FAILED");
        }
        robot.backLeft.setPower(0);

        encoderStart = robot.backRight.getCurrentPosition();
        robot.backRight.setPower(0.5);
        android.os.SystemClock.sleep(125);
        if (robot.backRight.getCurrentPosition() > encoderStart + 20) {
            telemetry.addLine("backRight encoder test passed");
        } else {
            telemetry.addLine("backRight encoder test FAILED");
        }
        robot.backRight.setPower(0);

        encoderStart = robot.frontLeft.getCurrentPosition();
        robot.frontLeft.setPower(0.5);
        android.os.SystemClock.sleep(125);
        if (robot.frontLeft.getCurrentPosition() > encoderStart + 20) {
            telemetry.addLine("frontLeft encoder test passed");
        } else {
            telemetry.addLine("frontLeft encoder test FAILED");
        }
        robot.frontLeft.setPower(0);

        // UPPER 0.47
        // LOWER 0.81
        encoderStart = robot.frontRight.getCurrentPosition();
        robot.frontRight.setPower(0.5);
        android.os.SystemClock.sleep(125);
        if (robot.frontRight.getCurrentPosition() > encoderStart + 20) {
            telemetry.addLine("frontRight encoder test passed");
        } else {
            telemetry.addLine("frontRight encoder test FAILED");
        }
        robot.frontRight.setPower(0);

        telemetry.update();
        android.os.SystemClock.sleep(1500);
        robot.slideRot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        while(opModeIsActive()) {
            // note: encoder for backLeft is currently broken
            telemetry.addData("backLeft encoder value:", robot.backLeft.getCurrentPosition());
            telemetry.addData("backRight encoder value:", robot.backRight.getCurrentPosition());
            telemetry.addData("frontLeft encoder value:", robot.frontLeft.getCurrentPosition());
            telemetry.addData("frontRight encoder value:", robot.frontRight.getCurrentPosition());
            telemetry.addData("slideExt encoder value:", robot.slideExt.getCurrentPosition());
            telemetry.addData("slideRot encoder value:", robot.slideRot.getCurrentPosition());
            telemetry.addData("gripper encoder value:" , robot.gripper.getPosition());
            telemetry.addData("imu yaw", robot.autoIMU.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
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
            if (robot.gamepad2.right_trigger > 0.05) {
                robot.gripper.setPosition(0.81);
                //if (!heldRight) {
                //    robot.gripper.setPosition(robot.gripper.getPosition() + 0.05);
                //    heldRight = true;
                //}
            } else if (robot.gamepad2.left_trigger > 0.05) {
                robot.gripper.setPosition(0.47);
                //if (!heldRight) {
                //    robot.gripper.setPosition(robot.gripper.getPosition() - 0.06);
                //    heldRight = true;
                //}
            } else {
                robot.gripper.setPosition(0.64);
                //heldRight = false;
            }
        }
    }
}
