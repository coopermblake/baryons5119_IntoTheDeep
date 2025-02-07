package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.core.Robot;
import org.firstinspires.ftc.teamcode.core.ViperSlide;

@Autonomous(name = "RR test", group = "Autonomous")
public class RR_Test extends LinearOpMode {

    @Config
    public static class AutoConstants{
        public static double y_1 = 24;
        public static double waitTime = 1.0;
        public static double x_1 = 24;

    }
    public void runOpMode(){
        Pose2d initialPose = new Pose2d(0,0, Math.toRadians(90));
        MecanumDrive mecanumDrive = new MecanumDrive(hardwareMap, initialPose);

        TrajectoryActionBuilder tab = mecanumDrive.actionBuilder(initialPose)
                .setTangent(Math.toRadians(90))
                .lineToY(AutoConstants.y_1)
                .waitSeconds(AutoConstants.waitTime)
                .setTangent(0)
                .lineToX(AutoConstants.x_1);

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
