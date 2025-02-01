package org.firstinspires.ftc.teamcode.opmodes.driverControl;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.core.Robot;

@TeleOp(name="Drive Mode",group="Competition Opmodes")
public class driveMode extends LinearOpMode {
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap, gamepad1, gamepad2);
        double heading;

        long lastCycleStart = System.currentTimeMillis();
        waitForStart();
        robot.viperSlide.driverControl = true;
        robot.teleOpIMU.resetYaw();
        while(opModeIsActive()) {
            telemetry.addData("refresh rate (hz): ",1000/(System.currentTimeMillis() - lastCycleStart + 1));
            lastCycleStart = System.currentTimeMillis();
            heading = robot.getYawRadians();
            robot.drivetrain.driveRobot(gamepad1, gamepad2, heading);
            robot.viperSlide.teleopSlideMovement(gamepad1, gamepad2);
            telemetry.addData("Heading radians:", heading);
            telemetry.addData("current macro", robot.viperSlide.currentMacro);
            telemetry.addData("field centric", robot.drivetrain.getFieldCentric());
            telemetry.addData("ext", robot.viperSlide.slideExt.getCurrentPosition());
            telemetry.addData("rot", robot.viperSlide.slideRot.getCurrentPosition());
            telemetry.addData("extMin", robot.viperSlide.extMin);
            telemetry.addData("extMaxLow", robot.viperSlide.extMaxLow);
            telemetry.addData("extMaxHigh", robot.viperSlide.extMaxHigh);
            telemetry.addData("rotMin", robot.viperSlide.rotMin);
            telemetry.addData("rotMax", robot.viperSlide.rotMax);
            telemetry.addData( "gripper", robot.gripper.getPosition());
            telemetry.addData("rightTrigger", gamepad2.right_trigger);
            telemetry.update();
        }
    }
}