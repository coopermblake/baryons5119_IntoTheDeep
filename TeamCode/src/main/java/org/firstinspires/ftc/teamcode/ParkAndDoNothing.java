package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "Park and do nothing", group = "Competition Opmodes")
public class ParkAndDoNothing extends LinearOpMode {
    public void runOpMode() throws InterruptedException {
        // TODO: huge hack, please replace this later
        Robot robot = new Robot(hardwareMap, gamepad1, gamepad2);
        robot.gripper.setPosition(0.81);
        waitForStart();
        robot.backLeft.setPower(0.5);
        robot.backRight.setPower(0.5);
        robot.frontLeft.setPower(0.5);
        robot.frontRight.setPower(0.5);
        android.os.SystemClock.sleep(100);

        robot.backLeft.setPower(-0.65);
        robot.backRight.setPower(0.65);
        robot.frontLeft.setPower(0.65);
        robot.frontRight.setPower(-0.65);

        android.os.SystemClock.sleep(1450);

        robot.backLeft.setPower(0);
        robot.backRight.setPower(0);
        robot.frontLeft.setPower(0);
        robot.frontRight.setPower(0);
    }
    
}
