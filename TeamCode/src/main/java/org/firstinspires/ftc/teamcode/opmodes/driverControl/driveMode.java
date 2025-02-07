package org.firstinspires.ftc.teamcode.opmodes.driverControl;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.core.Robot;

@TeleOp(name="Drive Mode",group="Competition Opmodes")
public class driveMode extends LinearOpMode {
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap, gamepad1, gamepad2);
        double headingRad;

        long lastCycleStart = System.currentTimeMillis();
        waitForStart();
        robot.viperSlide.driverControl = true;
        while(opModeIsActive()) {
            telemetry.addData("refresh rate (hz): ",1000/(System.currentTimeMillis() - lastCycleStart + 1));
            lastCycleStart = System.currentTimeMillis();
            headingRad = robot.getYawRadians();
            robot.drivetrain.driveRobot(gamepad1, gamepad2, headingRad);
            robot.viperSlide.teleopSlideMovement(gamepad1, gamepad2);
            
            telemetry.addLine("DRIVING");
            telemetry.addData("field centric", robot.drivetrain.getFieldCentric());
            telemetry.addData("Heading degrees:", robot.getYawDegrees());
            
            telemetry.addLine("ROTATION");
            telemetry.addData("rot macro", robot.viperSlide.rotateMacro);
            //telemetry.addData("rot Power", robot.viperSlide.slideRot.getPower());
            telemetry.addData("rot position", robot.viperSlide.slideRot.getCurrentPosition());
            telemetry.addData("rot target", robot.viperSlide.slideRot.getTargetPosition());
            //telemetry.addData("rot mode", robot.viperSlide.slideRot.getMode());
            //telemetry.addData("rot min", robot.viperSlide.rotMin);
            //telemetry.addData("rot max", robot.viperSlide.rotMax);

            telemetry.addLine("EXTENSION");
            telemetry.addData("ext macro", robot.viperSlide.extendMacro);
            //telemetry.addData("ext Power", robot.viperSlide.slideExt.getPower());
            telemetry.addData("ext position", robot.viperSlide.slideExt.getCurrentPosition());
            telemetry.addData("ext target", robot.viperSlide.slideExt.getTargetPosition());
            //telemetry.addData("ext mode", robot.viperSlide.slideExt.getMode());
            telemetry.addData("ext min", robot.viperSlide.extMin);
            telemetry.addData("ext max low", robot.viperSlide.extMaxLow);
            telemetry.addData("ext max high", robot.viperSlide.extMaxHigh);

            telemetry.addLine("GRIPPER");
            telemetry.addData( "gripper", robot.gripper.getPosition());

            telemetry.update();
        }
    }
}