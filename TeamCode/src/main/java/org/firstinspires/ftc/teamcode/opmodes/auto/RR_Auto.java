package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.core.ViperSlide;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

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

    }
    public void runOpMode(){
        Pose2d initialPose = new Pose2d(Acons._0_x, Acons._0_y, Acons._0_theta);

        MecanumDrive mecanumDrive = new MecanumDrive(hardwareMap, initialPose);
        ViperSlide viperSlide = new ViperSlide(hardwareMap.get(DcMotor.class, "slideExt"),
                                               hardwareMap.get(DcMotor.class, "slideRot"),
                                                gamepad1, gamepad2,
                                                hardwareMap.get(Servo.class, "gripper"));

        TrajectoryActionBuilder tab = mecanumDrive.actionBuilder(initialPose)
                .setTangent(Math.toRadians(90))
                .lineToY(Acons._1_y);


        //opMode starts here
        Actions.runBlocking(viperSlide.closeGripper());

        while(!isStopRequested() && opModeInInit()){
            telemetry.addLine("init");
            telemetry.addData("Pose", mecanumDrive.localizer.getPose());
        }

        waitForStart();

        if(isStopRequested()) return;

        Actions.runBlocking(
                tab.build()
        );

    }

}
