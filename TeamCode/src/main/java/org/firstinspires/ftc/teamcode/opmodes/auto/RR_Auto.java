package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;

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
        public static double _0_x = 0;
        public static double _0_y = -72;
        public static double _0_theta = Math.toRadians(90);
        public static double _1_y = -42;
        public static double _2_wait = 0.5;
        public static double _3_x = 24;
        public static double _4_x = 36;
        public static double _4_y = -33;
        public static double _4_h = Math.toRadians(-60);
        public static double _4_t = Math.toRadians(60);


    }
    public void runOpMode(){
        Pose2d initialPose = new Pose2d(Acons._0_x, Acons._0_y, Acons._0_theta);
        Pose2d pose_4= new Pose2d(Acons._4_x, Acons._4_y, Acons._4_h);



        MecanumDrive mecanumDrive = new MecanumDrive(hardwareMap, initialPose);

        TrajectoryActionBuilder tab = mecanumDrive.actionBuilder(initialPose)
                .setTangent(Math.toRadians(90))
                .lineToY(Acons._1_y)
                .waitSeconds(Acons._2_wait)
                .setTangent(Math.toRadians(180))
                .splineToSplineHeading(pose_4, Acons._4_t);

        while(!isStopRequested() && opModeInInit()){
            telemetry.addLine("init");
        }

        waitForStart();

        if(isStopRequested()) return;

        Actions.runBlocking(
                tab.build()
        );

    }

}
