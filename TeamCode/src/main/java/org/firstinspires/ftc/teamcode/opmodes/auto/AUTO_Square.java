package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.core.Robot;

@Autonomous
public class AUTO_Square extends LinearOpMode {


    public void runOpMode() throws InterruptedException{
        Robot robot = new Robot(hardwareMap, gamepad1, gamepad2);
        robot.initAutoDrive(telemetry);
        robot.imu.resetYaw();
        while(opModeInInit()){
            telemetry.addData(">", "Robot Heading = %4.0f", robot.getYawDegrees());
            telemetry.update();
        }

        robot.autoDrive.driveStraight(0.2, 24.0, 0.0);
        robot.autoDrive.turnToHeading(0.2, 90.0);
        robot.autoDrive.holdHeading(0.2, 90.0, 0.5);

        robot.autoDrive.driveStraight(0.2, 24.0, 90.0);

        robot.autoDrive.turnToHeading(0.2, 180.0);
        robot.autoDrive.holdHeading(0.2, 180.0, 0.5);

        robot.autoDrive.driveStraight(0.2, 24.0, 180.0);

        robot.autoDrive.turnToHeading(0.2, -90.0);
        robot.autoDrive.holdHeading(0.2, -90.0, 0.5);

        robot.autoDrive.driveStraight(0.2, 24.0, -90.0);
    }
}
