package org.firstinspires.ftc.teamcode.opmodes.auto;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


import org.firstinspires.ftc.teamcode.core.Robot;
import org.firstinspires.ftc.teamcode.lib.CustomPID;


@TeleOp(name="testingstrafe", group="Competition Opmodes")
public class testingstrafe extends LinearOpMode {




    public void runOpMode() throws InterruptedException{
        Robot robot = new Robot(hardwareMap, gamepad1, gamepad2);
        robot.initAutoDrive(telemetry);
        robot.imu.resetYaw();
        double kP = 0.0054;
        double kI = 0.0;
        double kD = 0.002;
        boolean debounce = false;
        while(opModeInInit()) {
            telemetry.addData(">", "Robot Heading = %4.0f", robot.getYawDegrees());
            telemetry.addData("100*kD", 100*kD);
            if (gamepad1.dpad_left || gamepad1.dpad_right ||
                    gamepad1.dpad_up || gamepad1.dpad_down) {
                if (!debounce) {
                    if (gamepad1.dpad_up) kD *= 10;
                    if (gamepad1.dpad_down) kD /= 10;
                    if (gamepad1.dpad_left) kD /= 1.5;
                    if (gamepad1.dpad_right) kD *= 1.5;
                    debounce = true;
                }
            } else {
                debounce = false;
                telemetry.update();
            }
        }




        robot.autoDrive.pidY = new CustomPID(kP, kI, kD);
       /*
       robot.frontLeft.setPower(0.3);
       robot.backLeft.setPower(0.3);
       robot.frontRight.setPower(0.3);
       robot.backRight.setPower(0.3);
       sleep(1000);


       robot.frontLeft.setPower(0);
       robot.backLeft.setPower(0);
       robot.frontRight.setPower(0);
       robot.backRight.setPower(0);
       */
        robot.autoDrive.driveStraight(0.3, 48, 0);
        sleep(1000);
        robot.autoDrive.driveStraight(0.3, -48, 0);
        sleep(1000);
        robot.autoDrive.driveStraight(0.3, 48, 0);
        sleep(1000);
        robot.autoDrive.driveStraight(0.3, -48, 0);
        sleep(1000);
        robot.autoDrive.driveStraight(0.3, 48, 0);
        sleep(1000);
        robot.autoDrive.driveStraight(0.3, -48, 0);
        sleep(1000);
//        robot.autoDrive.turnToHeading(0.3,90);
//        robot.autoDrive.holdHeading(0.3, 90,0.5);
//        sleep(1000);




        //robot.autoDrive.driveDiagonal(0.3, -24, 21, 10);
        //sleep(1000);
    }
}

