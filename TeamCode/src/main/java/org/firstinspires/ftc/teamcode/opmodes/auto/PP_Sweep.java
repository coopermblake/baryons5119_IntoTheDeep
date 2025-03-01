package org.firstinspires.ftc.teamcode.opmodes.auto;

import static org.firstinspires.ftc.teamcode.opmodes.auto.PPS.makePoint;
import static org.firstinspires.ftc.teamcode.opmodes.auto.PPS.makePose;

import com.pedropathing.follower.Follower;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.core.Sweeper;
import org.firstinspires.ftc.teamcode.core.ViperSlide;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.examples.GeneratedPaths;

@Autonomous(name = "pp sweep 1", group = "Competition Opmodes")
public class PP_Sweep extends OpMode {
    private Follower follower;
    ViperSlide viperSlide;
    Sweeper sweeper;

    Point home = new Point(PPS.home.x, PPS.home.y);

    private Path allign1, drag1, allign2, drag2, allign3, drag3, score2, grab3, score3, grab4, score4, grab5, score5;
    private PathChain scorePreload;
    private PathChain pcAllign1;
    private int pathState;
    private long pathStart;
    private long stepStart;

    private double getPathTime(){
        return (System.nanoTime() - pathStart)/1_000_000_000.0;
    }
    private double getStepTime(){
        return (System.nanoTime() - stepStart)/1_000_000_000.0;
    }

    public void buildPaths(){
        scorePreload = follower.pathBuilder()
                .addPath(new BezierLine(makePoint(PPS.home.class), makePoint(PPS.bar_1.class)))
                .setConstantHeadingInterpolation(PPS.home.h)
                .build();

        allign1 = new Path(new BezierCurve(makePoint(PPS.bar_r.class), makePoint(PPS.allign_1_c1.class), makePoint(PPS.allign_1_adj.class)));
        allign1.setLinearHeadingInterpolation(Math.toRadians(PPS.bar_r.h), Math.toRadians(PPS.allign_1_adj.h));
        allign1.setZeroPowerAccelerationMultiplier(PPS.zpams.allign1);


//        pcAllign1 = follower.pathBuilder()
//                .addPath(new BezierCurve(makePoint(PPS.bar_r.class), makePoint(PPS.allign_1_c1.class), makePoint(PPS.allign_1_adj.class)))
//                .setLinearHeadingInterpolation(PPS.bar_r.h, PPS.allign_1_adj.h)
//                .build();

        drag1 = new Path(new BezierLine(makePoint(PPS.allign_1_r.class), makePoint(PPS.drag_1_adj.class)));
        drag1.setLinearHeadingInterpolation(Math.toRadians(PPS.allign_1_r.h), Math.toRadians(PPS.drag_1_adj.h));
        drag1.setZeroPowerAccelerationMultiplier(PPS.zpams.drag1);

        allign2 = new Path(new BezierLine(makePoint(PPS.drag_1_r.class), makePoint(PPS.allign_2_adj.class)));
        allign2.setLinearHeadingInterpolation(Math.toRadians(PPS.drag_1_r.h), Math.toRadians(PPS.allign_2_adj.h));
        allign2.setZeroPowerAccelerationMultiplier(PPS.zpams.allign2);

        drag2 = new Path(new BezierLine(makePoint(PPS.allign_2_r.class), makePoint(PPS.drag_2_adj.class)));
        drag2.setLinearHeadingInterpolation(Math.toRadians(PPS.allign_2_r.h), Math.toRadians(PPS.drag_2_adj.h));
        drag2.setZeroPowerAccelerationMultiplier(PPS.zpams.drag2);

        allign3 = new Path(new BezierLine(makePoint(PPS.drag_2_r.class), makePoint(PPS.allign_3_adj.class)));
        allign3.setLinearHeadingInterpolation(Math.toRadians(PPS.drag_2_r.h), Math.toRadians(PPS.allign_3_adj.h));
        allign3.setZeroPowerAccelerationMultiplier(PPS.zpams.allign3);

        drag3 = new Path(new BezierLine(makePoint(PPS.allign_3_r.class), makePoint(PPS.drag_3_adj.class)));
        drag3.setLinearHeadingInterpolation(Math.toRadians(PPS.allign_3_r.h), Math.toRadians(PPS.drag_3_adj.h));
        drag3.setZeroPowerAccelerationMultiplier(PPS.zpams.drag3);

        score2 = new Path(new BezierCurve(makePoint(PPS.drag_3_r.class), makePoint(PPS.hang_c2.class), makePoint(PPS.bar_2.class)));

        grab3 = new Path(new BezierCurve(makePoint(PPS.bar_r.class), makePoint(PPS.grab_c3.class), makePoint(PPS.grab_3.class)));
        score3 = new Path(new BezierCurve(makePoint(PPS.grab_r.class), makePoint(PPS.hang_c3.class), makePoint(PPS.bar_3.class)));

        grab4 = new Path(new BezierCurve(makePoint(PPS.bar_r.class), makePoint(PPS.grab_c4.class), makePoint(PPS.grab_4.class)));
        score4 = new Path(new BezierCurve(makePoint(PPS.grab_r.class), makePoint(PPS.hang_c4.class), makePoint(PPS.bar_4.class)));

        grab5 = new Path(new BezierCurve(makePoint(PPS.bar_r.class), makePoint(PPS.grab_c5.class), makePoint(PPS.grab_5.class)));
        score5 = new Path(new BezierCurve(makePoint(PPS.grab_r.class), makePoint(PPS.hang_c5.class), makePoint(PPS.bar_5.class)));
    }

    public void autonomousPathUpdate(){
        switch(pathState){
            case 0:
                pathStart = System.nanoTime();
                stepStart = System.nanoTime();
                setPathState(1);
            case 1:
                //follower.followPath(scorePreload);
                viperSlide.rotHang();
                viperSlide.extPreHang();
                if(getStepTime() > PPS.delays.startDelay) {
                    setPathState(2);
                }
                break;
            case 2:
                follower.followPath(scorePreload);
                setPathState(3);
                break;
            case 3:
                if(!follower.isBusy()) {
                    if (viperSlide.extPostHang()) {
                        viperSlide.openGripper();
                        setPathState(4);
                    }
                }
                break;
            case 4:
                if(getStepTime() > PPS.delays.hangPostDelay){
                    viperSlide.extHome();
                    follower.followPath(allign1);
                    setPathState(5);
                }
                break;
            case 5:
                if (!follower.isBusy() && getStepTime() > PPS.delays.allignSweeperDelay) {
                    sweeper.deploy();
                }
                if(sweeper.sweeper.getPosition() < 0.1){
                    setPathState(6);
                }
                break;
            case 6:
                if(getStepTime() > PPS.delays.sweeperDeployDelay){
                    follower.followPath(drag1);
                    setPathState(7);
                }
                break;
            case 7:
                if(!follower.isBusy()){
                    sweeper.raise();
                }
                if(sweeper.sweeper.getPosition()>0.2){
                    setPathState(8);
                }
                break;
            case 8:
                if(getStepTime() > PPS.delays.sweeperDeployDelay){
                    follower.followPath(allign2);
                    setPathState(9);
                }
                break;
            case 9:
                if (!follower.isBusy() && getStepTime() > PPS.delays.allignSweeperDelay) {
                    sweeper.deploy();
                }
                if(sweeper.sweeper.getPosition() < 0.1){
                    setPathState(10);
                }
                break;
            case 10:
                if(getStepTime() > PPS.delays.sweeperDeployDelay){
                    follower.followPath(drag2);
                    setPathState(11);
                }
                break;
            case 11:
                if(!follower.isBusy()){
                    sweeper.raise();
                }
                if(sweeper.sweeper.getPosition()>0.2){
                    setPathState(12);
                }
                break;
            case 12:
                if(getStepTime() > PPS.delays.sweeperDeployDelay){
                    follower.followPath(allign3);
                    setPathState(13);
                }
                break;
            case 13:
                if (!follower.isBusy() && getStepTime() > PPS.delays.allignSweeperDelay) {
                    sweeper.deploy();
                }
                if(sweeper.sweeper.getPosition() < 0.1){
                    setPathState(14);
                }
                break;
            case 14:
                if(getStepTime() > PPS.delays.sweeperDeployDelay){
                    follower.followPath(drag3);
                    setPathState(15);
                }
                break;
            case 15:
                if (!follower.isBusy()){
                    sweeper.raise();
                }
                if(sweeper.sweeper.getPosition()>0.2){
                    setPathState(16);
                }
                break;
        }
    }

    public void setPathState(int pState){
        pathState = pState;
        stepStart = System.nanoTime();
    }

    @Override
    public void init() {
        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(makePose(PPS.home.class));
        buildPaths();
        viperSlide = new ViperSlide(hardwareMap);
        viperSlide.closeGripper();
        sweeper = new Sweeper(hardwareMap.get(Servo.class, "sweeper"));
        sweeper.retract();
    }

    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();
        telemetry.addData("Path State", pathState);
        telemetry.addData("Position", follower.getPose().toString());
        telemetry.addData("Path Time", getPathTime());
        telemetry.addData("Step Time", getStepTime());
        telemetry.update();
    }
}
