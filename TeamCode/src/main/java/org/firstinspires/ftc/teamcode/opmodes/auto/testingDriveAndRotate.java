package org.firstinspires.ftc.teamcode.opmodes.auto;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


import org.firstinspires.ftc.teamcode.core.Robot;
import org.firstinspires.ftc.teamcode.lib.CustomPID;


@TeleOp(name="testing drive and rotate", group="Competition Opmodes")
public class testingDriveAndRotate extends LinearOpMode {




    public void runOpMode() throws InterruptedException{
        Robot robot = new Robot(hardwareMap, gamepad1, gamepad2);
        robot.initAutoDrive(telemetry);
        robot.imu.resetYaw();

        while(opModeInInit()) {
            telemetry.addData(">", "Robot Heading = %4.0f", robot.getYawDegrees());

        }

        telemetry.addLine("Forward");
        telemetry.update();
        robot.autoDrive.driveAndRotate(0.2, 0, 24, 0, 0, 10);

        sleep(2000);
        telemetry.addLine("Backward");
        telemetry.update();
        robot.autoDrive.driveAndRotate(0.2, 0, -24, 0, 0, 10);

        sleep(2000);

        telemetry.addLine("Left");
        telemetry.update();
        robot.autoDrive.driveAndRotate(0.2, -24, 0, 0, 0, 10);

        sleep(2000);

        telemetry.addLine("Right");
        telemetry.update();
        robot.autoDrive.driveAndRotate(0.2, 24, 0, 0, 0, 10);

        sleep(2000);
//
//        telemetry.addLine("Forward and Rotate");
//        telemetry.update();
//        robot.autoDrive.driveAndRotate(0.2, 0, 24, 0.2, 180, 10);

//        sleep(2000);
//
//        telemetry.addLine("Backward and Rotate");
//        telemetry.update();
////        robot.autoDrive.driveAndRotate(0.2, 0, -24, 0.2, 90, 10);
//        sleep(2000);
//
//        telemetry.addLine("Left and Rotate");
//        telemetry.update();
////        robot.autoDrive.driveAndRotate(0.2, -24, 0, 0.2, 180, 10);
//        sleep(2000);
//
//        telemetry.addLine("Right and Rotate");

//        telemetry.update();
//        robot.autoDrive.driveAndRotate(0.2, 24, 0, 0.2, 0, 10);

//
//        sleep(2000);
//
//        telemetry.addLine("Diagonal");
//        telemetry.update();
//        robot.autoDrive.driveAndRotate(0.2, 24, 24, 0, 0, 10);

//
//        sleep(2000);
//
//        telemetry.addLine("Reverse Diagonal");
//        telemetry.update();
        //        robot.autoDrive.driveAndRotate(0.2, 24, 24, 0, 0, 10);
        //sleep(2000);

        //telemetry.addLine("diagonal and rotate");
        //telemetry.update();
        //        robot.autoDrive.driveAndRotate(0.2, 24, 24, 0.2, 180, 10);

//        sleep(2000);


        //telemetry.addLine("diagonal and rotate");
        //telemetry.update();
        //        robot.autoDrive.driveAndRotate(0.2, -24, -24, 0.2, 90, 10);

//        sleep(2000);



    }
}

