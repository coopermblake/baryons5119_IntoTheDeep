package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Trajectory;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TurnConstraints;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.core.ViperSlide;

@Autonomous(name = "RR dragging auto")
public class RR_Dragger extends LinearOpMode {

    @Config
    public static class delays{
        public static double grabPreDelay = 1;
        public static double grabPostDelay = 0.5;
        public static double grabRaiseMoveDelay = 0.5;
        public static double dragExtendDelay = 1.3;
        public static double hangPostDelay;
        public static double hangPreDelay;
        public static double dragRaiseDelay = 0.5;
        public static double endDelay = 5;
    }

    @Config
    public static class start{
        public static double x = 8;
        public static double y = -63.5;
        public static double h = 90;

    }

    @Config
    public static class hang1{
        public static double y = -35;
    }

    @Config
    public static class dragging1{
        public static double Ast = 270;
        public static double Aet = 0;
        public static double Ax = 26;
        public static double Ay = -38;
        public static double Ah = 45;
        public static double Bh = -60;
        public static double B_vel = Math.PI/2;
        public static double BminAcc = -Math.PI/2;
        public static double BmaxAcc = Math.PI/2;
    }

    @Config
    public static class dragging2{
        public static double Ast = 0;
        public static double Aet = 0;
        public static double Ax = dragging1.Ax+10;
        public static double Ay = dragging1.Ay;
        public static double Ah = dragging1.Ah;
        public static double Bh = dragging1.Bh;
        public static double B_vel = Math.PI/2;
        public static double BminAcc = -Math.PI/2;
        public static double BmaxAcc = Math.PI/2;
    }

    @Config
    public static class grabbing2{
        public static int x = 38;
        public static int y = -54;
        public static int h = 270;
        public static int st = -60;
        public static int et = 270;
        public static double turnVel = Math.PI/2;
        public static double turnMinAcc = -Math.PI/2;
        public static double turnMaxAcc = Math.PI/2;
    }

    @Config
    public static class hanging2{
        public static double x = start.x;
        public static double y = hang1.y;
        public static double h = 90;
        public static double st = 180;
        public static double et = 90;
    }

    public void runOpMode() {
        Pose2d initialPose = new Pose2d(start.x, start.y, Math.toRadians(start.h));

        MecanumDrive mecanumDrive = new MecanumDrive(hardwareMap, initialPose);
        ViperSlide viperSlide = new ViperSlide(hardwareMap.get(DcMotor.class, "slideExt"),
                hardwareMap.get(DcMotor.class, "slideRot"),
                gamepad1, gamepad2,
                hardwareMap.get(Servo.class, "gripper"));

        TrajectoryActionBuilder startToBar = mecanumDrive.actionBuilder(initialPose)
                .waitSeconds(0.5)
                .setTangent(Math.toRadians(90))
                .lineToY(RR_Auto.Acons._1_y);



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

        Actions.runBlocking(new SequentialAction(
                new ParallelAction(
                        viperSlide.extendToHang(),
                        viperSlide.rotateUp(),
                        startToBar.build()
                ),

                viperSlide.retractToHang(),
                viperSlide.openGripper(),
                new SleepAction(delays.grabRaiseMoveDelay),
                new ParallelAction(
                        barToDrag1(mecanumDrive).build(),
                        viperSlide.rotateToDrag(),
                        new SequentialAction(
                                new SleepAction(delays.dragExtendDelay),
                                viperSlide.extendToDrag()
                        )
                ),
                dragTurn1(mecanumDrive).build(),
                new ParallelAction(
                        viperSlide.rotateHorizontal(),
                        drag2(mecanumDrive).build()
                ),
                viperSlide.rotateToDrag(),
                dragTurn2(mecanumDrive).build(),
                new ParallelAction(
                        viperSlide.rotateHorizontal(),
                        viperSlide.extendToHome(),
                        new SequentialAction(
                                new SleepAction(delays.dragRaiseDelay),
                                grab1(mecanumDrive).build()
                        )
                ),
                viperSlide.extendToGrab(),
                viperSlide.closeGripper(),
                new ParallelAction(
                        viperSlide.rotateUp(),
                        new SequentialAction(
                                new SleepAction(RR_Auto.Acons.grabRaiseMoveDelay),
                                viperSlide.extendToHang()
                        ),
                        new SequentialAction(
                                new SleepAction(RR_Auto.Acons.grabRaiseMoveDelay),
                                hang2(mecanumDrive).build()

                        )
                ),

                new SleepAction(delays.endDelay)
                )


        );

    }

    private TrajectoryActionBuilder barToDrag1(MecanumDrive mecanumDrive) {
        return mecanumDrive.actionBuilder(new Pose2d(start.x, hang1.y, start.h))
                .setTangent(Math.toRadians(dragging1.Ast))
                .splineToSplineHeading(new Pose2d(dragging1.Ax, dragging1.Ay, Math.toRadians(dragging1.Ah)), Math.toRadians(dragging1.Aet));

    }

    private TrajectoryActionBuilder dragTurn1(MecanumDrive mecanumDrive){
        return mecanumDrive.actionBuilder(new Pose2d(dragging1.Ax, dragging1.Ay, Math.toRadians(dragging1.Ah)))
                .turnTo(Math.toRadians(dragging1.Bh),
                new TurnConstraints(dragging1.B_vel, dragging1.BminAcc, dragging1.BmaxAcc));
    }

    private TrajectoryActionBuilder drag2(MecanumDrive mecanumDrive){
        return mecanumDrive.actionBuilder(new Pose2d(dragging1.Ax, dragging1.Ay, Math.toRadians(dragging1.Bh)))
                .setTangent(dragging2.Ast)
                .splineToSplineHeading(new Pose2d(dragging2.Ax, dragging2.Ay, Math.toRadians(dragging2.Ah)), Math.toRadians(dragging2.Aet));
    }

    private TrajectoryActionBuilder dragTurn2(MecanumDrive mecanumDrive){
        return mecanumDrive.actionBuilder(new Pose2d(dragging2.Ax, dragging2.Ay, Math.toRadians(dragging2.Ah)))
                .turnTo(Math.toRadians(dragging2.Bh), new TurnConstraints(dragging2.B_vel, dragging2.BminAcc, dragging2.BmaxAcc));
    }

    private TrajectoryActionBuilder grab1(MecanumDrive mecanumDrive){
        return mecanumDrive.actionBuilder(new Pose2d(mecanumDrive.localizer.getPose().position.x,
                                                    mecanumDrive.localizer.getPose().position.y,
                                                   mecanumDrive.localizer.getPose().heading.toDouble()))
                .setTangent(Math.toRadians(grabbing2.st))
                .splineToSplineHeading(new Pose2d(grabbing2.x, grabbing2.y, Math.toRadians(grabbing2.h)), Math.toRadians(grabbing2.et));
    }

    private TrajectoryActionBuilder hang2(MecanumDrive mecanumDrive){
        return mecanumDrive.actionBuilder(mecanumDrive.localizer.getPose())
                .setTangent(Math.toRadians(hanging2.st))
                .splineToSplineHeading(new Pose2d(hanging2.x, hanging2.y, Math.toRadians(hanging2.h)), Math.toRadians(hanging2.et));
    }
}
