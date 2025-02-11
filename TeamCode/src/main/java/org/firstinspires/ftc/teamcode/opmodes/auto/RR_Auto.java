package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.core.ViperSlide;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.Vector;

@Autonomous(name = "RR Auto", group = "Autonomous")
public class RR_Auto extends LinearOpMode {

    /*
    auto steps
    1. drive forward to bar
    2. hang specimen
    3. drive right
    4. curve around bar support
    5. curve around sample
    6. push sample
     */

    @Config
    public static class Acons {//Auto Constants
        public static double grabPreDelay = 1;
        public static double grabPostDelay = 0.5;
        public static double grabRaiseMoveDelay = 0.5;

        //start
        public static double _0_x = 8;
        public static double _0_y = -63.5;
        public static double _0_theta = 90;

        //drive to sub
        public static double _1_y = -33;

        //drive behind sub
        public static double _2_x= 28;
        public static double _2_y = -36;
        public static double _2_et = 0;
        public static double _2_st = 270;
        public static double _2_arm_wait = 1;

        //drive next to sample
        public static double _3_x = 36;
        public static double _3_y = -24;
        public static double _3_t = 90;


        //drive behind sample
        public static double _4_x = 52;
        public static double _4_y = 0;
        public static double _4_t = 90;

        //turn to OZ
        public static double _5_h = -90;

        //drive to OZ
        public static double _6_y = -53;

        //driving to hang spec 2
        public static double _7_x = 0;
        public static double _7_y = -31;
        public static double _7_h = 90;
        public static double _7_st = 60;
        public static double _7_et = 90;

        //driving back to OZ waypoint
        public static double _8_x = 24;
        public static double _8_y = -40;
        public static double _8_h = -30;
        public static double _8_st = -45;
        public static double _8_et = 0;

        //back at OZ
        public static double _9_x = 48;
        public static double _9_y = -48;
        public static double _9_h = 270;
        public static double _9_st = 0;
        public static double _9_et = 270;

        //driving to hang spec 3
        public static double _10_x = 0;
        public static double _10_y = -31;
        public static double _10_h = 90;
        public static double _10_st = 150;
        public static double _10_et = 90;

        //park
        public static double _11_x = 56;
        public static double _11_y = -60;
        public static double _11_st = -60;
        public static double _11_et = 270;

    }
    public void runOpMode(){
        Pose2d initialPose = new Pose2d(Acons._0_x, Acons._0_y, Math.toRadians(Acons._0_theta));
        //Pose2d pose7 = new Pose2d(Acons._7_x, Acons._7_y, Math.toRadians(Acons._7_h));

        MecanumDrive mecanumDrive = new MecanumDrive(hardwareMap, initialPose);
        ViperSlide viperSlide = new ViperSlide(hardwareMap.get(DcMotor.class, "slideExt"),
                                               hardwareMap.get(DcMotor.class, "slideRot"),
                                                gamepad1, gamepad2,
                                                hardwareMap.get(Servo.class, "gripper"));


        TrajectoryActionBuilder startToBar = mecanumDrive.actionBuilder(initialPose)
                .waitSeconds(0.5)
                .setTangent(Math.toRadians(90))
                .lineToY(Acons._1_y);


        //opMode starts here

        while(!isStopRequested() && opModeInInit()){
            telemetry.addLine("init");
            telemetry.addData("X", mecanumDrive.localizer.getPose().position.x);
            telemetry.addData("Y", mecanumDrive.localizer.getPose().position.y);
            telemetry.update();
            //telemetry.addData("Heading", mecanumDrive.localizer.getPose().heading.toDouble());
            Actions.runBlocking(viperSlide.closeGripper());
        }

        waitForStart();

        if(isStopRequested()) return;
        viperSlide.reverseExt();
        viperSlide.resetEncoder();
        Actions.runBlocking(
                    new SequentialAction(
                            //raise arm and drive to bar

                            new ParallelAction(
                                    viperSlide.extendToHang(),
                                    viperSlide.rotateUp(),
                                    startToBar.build()
                            ),

                            viperSlide.retractToHang(),
                            viperSlide.openGripper(),

                            new ParallelAction(
                                    pushPieceToOZ(mecanumDrive).build(),
                                    new SequentialAction(
                                            new SleepAction(Acons._2_arm_wait),
                                            viperSlide.extendToHome(),
                                            viperSlide.rotateHorizontal()
                                    )

                            ),

                            new SleepAction(Acons.grabPreDelay),
                            viperSlide.extendToGrab(),
                            viperSlide.closeGripper(),
                            new SleepAction(Acons.grabPostDelay),

                            new ParallelAction(
                                    viperSlide.rotateUp(),
                                    new SequentialAction(
                                            new SleepAction(Acons.grabRaiseMoveDelay),
                                            viperSlide.extendToHang()
                                    ),
                                    new SequentialAction(
                                            new SleepAction(Acons.grabRaiseMoveDelay),
                                            OZToBar1(mecanumDrive).build()

                                    )
                            ),

                            new ParallelAction(
                                    viperSlide.retractToHang(),
                                    viperSlide.rotateLock()
                            ),
                            viperSlide.openGripper(),
                            new SleepAction(Acons.grabRaiseMoveDelay),

                            new ParallelAction(
                                barToOZ(mecanumDrive).build(),
                                new SequentialAction(
                                    new SleepAction(Acons._2_arm_wait),
                                    viperSlide.extendToHome(),
                                    viperSlide.rotateHorizontal()
                                )
                            ),

                            new SleepAction(Acons.grabPreDelay),
                            viperSlide.extendToGrab(),
                            viperSlide.closeGripper(),
                            new SleepAction(Acons.grabPostDelay),

                            new ParallelAction(
                                    viperSlide.rotateUp(),
                                    new SequentialAction(
                                            new SleepAction(Acons.grabRaiseMoveDelay),
                                            viperSlide.extendToHang()
                                    ),
                                    new SequentialAction(
                                            new SleepAction(Acons.grabRaiseMoveDelay),
                                            OZToBar1(mecanumDrive).build()

                                    )
                            ),

                            new ParallelAction(
                                    viperSlide.retractToHang(),
                                    viperSlide.rotateLock()
                            ),
                            viperSlide.openGripper(),
                            new SleepAction(Acons.grabRaiseMoveDelay),
                            park(mecanumDrive).build()

                )

        );

        telemetry.addLine("done");
        telemetry.addData("current", viperSlide.slideExt.getCurrentPosition());
        telemetry.addData("min", viperSlide.extMin);
        telemetry.update();
        sleep(10000);

    }

    private TrajectoryActionBuilder OZToBar2(MecanumDrive mecanumDrive) {
        TrajectoryActionBuilder tab = mecanumDrive.actionBuilder(mecanumDrive.localizer.getPose())
                .setTangent(Math.toRadians(Acons._10_st))
                .splineToLinearHeading(new Pose2d(Acons._10_x, Acons._10_y, Math.toRadians(Acons._10_h)), Math.toRadians(Acons._10_et));
        return tab;
    }

    private TrajectoryActionBuilder OZToBar1(MecanumDrive mecanumDrive){
        TrajectoryActionBuilder tab = mecanumDrive.actionBuilder(mecanumDrive.localizer.getPose())
                .setTangent(Math.toRadians(Acons._7_st))
                .splineToSplineHeading(new Pose2d(Acons._7_x, Acons._7_y, Math.toRadians(Acons._7_h)), Math.toRadians(Acons._7_et));
        return tab;
    }

    private TrajectoryActionBuilder barToOZ(MecanumDrive mecanumDrive){
        TrajectoryActionBuilder tab = mecanumDrive.actionBuilder(mecanumDrive.localizer.getPose())
                .setTangent(Math.toRadians(Acons._8_st))
                .splineToConstantHeading(new Vector2d(Acons._8_x, Acons._8_y), Math.toRadians(Acons._8_et))
                .splineToSplineHeading(new Pose2d(Acons._9_x, Acons._9_y, Math.toRadians(Acons._9_h)), Math.toRadians(Acons._9_et));
        return tab;
    }

    private TrajectoryActionBuilder pushPieceToOZ(MecanumDrive mecanumDrive){
        TrajectoryActionBuilder tab = mecanumDrive.actionBuilder(mecanumDrive.localizer.getPose())
                .setTangent(Acons._2_st)
                .splineToConstantHeading(new Vector2d(Acons._2_x, Acons._2_y), Math.toRadians(Acons._2_et))
                .splineToConstantHeading(new Vector2d(Acons._3_x, Acons._3_y), Math.toRadians(Acons._3_t))
                .splineToConstantHeading(new Vector2d(Acons._4_x, Acons._4_y), Math.toRadians(Acons._4_t))
                .turnTo(Math.toRadians(Acons._5_h))
                .lineToY(Acons._6_y);
        return tab;
    }

    private TrajectoryActionBuilder park(MecanumDrive mecanumDrive){
        TrajectoryActionBuilder tab = mecanumDrive.actionBuilder(mecanumDrive.localizer.getPose())
                .setTangent(Acons._11_st)
                .splineToConstantHeading(new Vector2d(Acons._11_x, Acons._11_y), Acons._11_et);
        return tab;
    }




}
