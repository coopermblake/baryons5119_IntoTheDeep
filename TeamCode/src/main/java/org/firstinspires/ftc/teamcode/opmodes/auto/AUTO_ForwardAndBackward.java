//package org.firstinspires.ftc.teamcode.opmodes.auto;
//
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import org.firstinspires.ftc.teamcode.core.Robot;
//
//@Disablefd
//public class AUTO_ForwardAndBackward extends LinearOpMode {
//
//    Robot robot = new Robot(hardwareMap, gamepad1, gamepad2);
//
//    public void runOpMode throws InterruptedException() {
//        while(opModeInInit()) {
//            telemetry.addData(">", "Robot Heading = %4.0f", robot.getYawDegrees());
//            telemetry.update();
//        }
//        //1
//        robot.autoDrive.driveStraight(0.4, 24.0, 0.0);
//        sleep(500);
//        robot.autoDrive.driveStraight(-0.4, 24.0,0.0);
//        sleep(500);
//
//        //2
//        robot.autoDrive.driveStraight(0.4, 24.0, 0.0);
//        sleep(500);
//        robot.autoDrive.driveStraight(-0.4, 24.0,0.0);
//        sleep(500);
//
//        //3
//        robot.autoDrive.driveStraight(0.4, 24.0, 0.0);
//        sleep(500);
//        robot.autoDrive.driveStraight(-0.4, 24.0,0.0);
//        sleep(500);
//
//        //4
//        robot.autoDrive.driveStraight(0.4, 24.0, 0.0);
//        sleep(500);
//        robot.autoDrive.driveStraight(-0.4, 24.0,0.0);
//        sleep(500);
//
//        //5
//        robot.autoDrive.driveStraight(0.4, 24.0, 0.0);
//        sleep(500);
//        robot.autoDrive.driveStraight(-0.4, 24.0,0.0);
//        sleep(500);
//
//        //6
//        robot.autoDrive.driveStraight(0.4, 24.0, 0.0);
//        sleep(500);
//        robot.autoDrive.driveStraight(-0.4, 24.0,0.0);
//        sleep(500);
//    }
//}
//
////    public void start(){
////        //1
////        robot.autoDrive.driveStraight(0.4, 24.0, 0.0);
////        sleep(500);
////        robot.autoDrive.driveStraight(-0.4, 24.0,0.0);
////        sleep(500);
////
////        //2
////        robot.autoDrive.driveStraight(0.4, 24.0, 0.0);
////        sleep(500);
////        robot.autoDrive.driveStraight(-0.4, 24.0,0.0);
////        sleep(500);
////
////        //3
////        robot.autoDrive.driveStraight(0.4, 24.0, 0.0);
////        sleep(500);
////        robot.autoDrive.driveStraight(-0.4, 24.0,0.0);
////        sleep(500);
////
////        //4
////        robot.autoDrive.driveStraight(0.4, 24.0, 0.0);
////        sleep(500);
////        robot.autoDrive.driveStraight(-0.4, 24.0,0.0);
////        sleep(500);
////
////        //5
////        robot.autoDrive.driveStraight(0.4, 24.0, 0.0);
////        sleep(500);
////        robot.autoDrive.driveStraight(-0.4, 24.0,0.0);
////        sleep(500);
////
////        //6
////        robot.autoDrive.driveStraight(0.4, 24.0, 0.0);
////        sleep(500);
////        robot.autoDrive.driveStraight(-0.4, 24.0,0.0);
////        sleep(500);
////    }
////
////    public void loop() {
////        for (String i : robot.autoDrive.autoDriveTelemetry) {
////            telemetry.addLine(i);
////        }
////        telemetry.update();
////    }
////
////
////    public void stop() {
////        robot.frontLeft.setPower(0);
////        robot.frontLeft.setPower(0);
////        robot.frontLeft.setPower(0);
////        robot.frontLeft.setPower(0);
////
////        robot.viperSlide.slideExt.setPower(0);
////        robot.viperSlide.slideRot.setPower(0);
////    }
////
////    public void sleep(int ms){
////        android.os.SystemClock.sleep(ms);
////    }
////
////}
