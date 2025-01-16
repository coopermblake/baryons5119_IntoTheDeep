package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.core.Robot;

@Autonomous
public class AUTO_Square extends OpMode {

    Robot robot = new Robot(hardwareMap, gamepad1, gamepad2);
    public void init(){

    }

    public void init_loop() {
        telemetry.addData(">", "Robot Heading = %4.0f", robot.getYawDegrees());
        telemetry.update();
    }

    public void start(){
        robot.autoDrive.driveStraight(0.4, 24.0, 0.0);
        slep(500);

        robot.autoDrive.turnToHeading(0.2, 90.0);
        robot.autoDrive.holdHeading(0.2, 90.0, 0.5);

        robot.autoDrive.driveStraight(0.4, 24.0, 90.0);
        slep(500);

        robot.autoDrive.turnToHeading(0.2, 180.0);
        robot.autoDrive.holdHeading(0.2, 180.0, 0.5);

        robot.autoDrive.driveStraight(0.4, 24.0, 180.0);
        slep(500);

        robot.autoDrive.turnToHeading(0.2, -90.0);
        robot.autoDrive.holdHeading(0.2, -90.0, 0.5);

        robot.autoDrive.driveStraight(0.4, 24.0, -90.0);
        slep(500);
    }

    public void loop() {
        for (String i : robot.autoDrive.autoDriveTelemetry) {
            telemetry.addLine(i);
        }
        telemetry.update();
    }


    public void stop() {
        robot.frontLeft.setPower(0);
        robot.frontLeft.setPower(0);
        robot.frontLeft.setPower(0);
        robot.frontLeft.setPower(0);

        robot.viperSlide.slideExt.setPower(0);
        robot.viperSlide.slideRot.setPower(0);
    }

    public void slep(int ms){
        android.os.SystemClock.sleep(ms);
    }

}
