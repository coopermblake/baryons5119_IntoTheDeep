package org.firstinspires.ftc.teamcode.opmodes.auto;

import static org.firstinspires.ftc.teamcode.opmodes.auto.PPP.*;

import com.pedropathing.follower.Follower;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.core.ViperSlide;

@Autonomous(name = "PP push", group = "Competition Opmodes")
public class PP_Push extends OpMode {
    private Follower follower;
    ViperSlide viperSlide;
    private PathChain score1, push1, score2, pick3, score3, park;
    private int pathState;
    private long pathStart;
    private long stepStart;

    private double getPathTime(){
        return (System.nanoTime() - pathStart)/1_000_000_000.0;
    }
    private double getStepTime(){
        return (System.nanoTime() - stepStart)/1_000_000_000.0;
    }
    private void buildPaths(){
        score1 = follower.pathBuilder()
                .addPath(new BezierLine(makePoint(start.class), makePoint(score1_adj.class)))
                .setConstantHeadingInterpolation(Math.toRadians(start.h))
                .build();
        push1 = follower.pathBuilder()
                .addPath(new BezierCurve(makePoint(score1_r.class), makePoint(nextToBar_c1.class), makePoint(nextToBar_adj.class)))
                .setLinearHeadingInterpolation(Math.toRadians(score1_r.h), Math.toRadians(nextToBar_adj.h))
                .addPath(new BezierCurve(makePoint(nextToBar_r.class), makePoint(push1_c1.class), makePoint(push1_adj.class)))
                .setConstantHeadingInterpolation(Math.toRadians(nextToBar_r.h))
                .addPath(new BezierLine(makePoint(push1_r.class), makePoint(pickup2_adj.class)))
                .setConstantHeadingInterpolation(Math.toRadians(push1_r.h))
                .build();
        score2 = follower.pathBuilder()
                .addPath(new BezierCurve(makePoint(pickup2_r.class), makePoint(score2_c1.class), makePoint(score2_adj.class)))
                .setLinearHeadingInterpolation(Math.toRadians(pickup2_r.h), Math.toRadians(score2_adj.h))
                .build();
        pick3 = follower.pathBuilder()
                .addPath(new BezierCurve(makePoint(score2_r.class), makePoint(pickup3_c1.class), makePoint(pickup3_adj.class)))
                .setLinearHeadingInterpolation(Math.toRadians(score2_r.h), Math.toRadians(pickup3_adj.h))
                .build();
        score3 = follower.pathBuilder()
                .addPath(new BezierCurve(makePoint(pickup3_r.class), makePoint(score3_c1.class), makePoint(score3_adj.class)))
                .setLinearHeadingInterpolation(Math.toRadians(pickup3_r.h), Math.toRadians(score3_adj.h))
                .build();
        park = follower.pathBuilder()
                .addPath(new BezierCurve(makePoint(score3_r.class), makePoint(park_c1.class), makePoint(park_r.class)))
                .setConstantHeadingInterpolation(Math.toRadians(score3_r.h))
                .build();


    }

    private void autonomousPathUpdate(){
        switch(pathState){
            case 0:
                //raise arm and extend
                viperSlide.rotHang();
                viperSlide.extPreHang();
                setPathState(-1);
                break;

            case 1:
                //drive to bar
                if(getStepTime() > delays.start_raise){
                    follower.followPath(score1, true);
                    setPathState(-1);
                }
                break;
            case 2:
                //retract arm to score first specimen
                if(!follower.isBusy() && viperSlide.rotHang() && viperSlide.extPreHang()){
                    viperSlide.extPostHang();
                    setPathState(-1);
                }
                break;
            case 3:
                //release sample
                if(viperSlide.extPostHang() && getStepTime() > delays.score_pre){
                    viperSlide.openGripper();
                    setPathState(-1);
                }
                break;
            case 4:
                //slightly raise arm away from bar

                if(getStepTime() > delays.score_post){
                    if(viperSlide.rotAwayFromBar()) {
                        setPathState(-1);
                    }
                }
                break;
            case 5:
                //retract arm
                //lower arm to grabbing position
                //push first sample to OZ
                viperSlide.extHome();
                viperSlide.rotHorPre();
                follower.followPath(push1, false);
                if(!follower.isBusy() && viperSlide.extHome() && viperSlide.rotHorPre()){
                    setPathState(-1);
                }
                break;
            case 6:
                //extend to grab
                if(getStepTime() > delays.grab_pre){
                    viperSlide.extHor();
                }
                break;
            case 7:
                //grab specimen
                if(viperSlide.extHor()){
                    viperSlide.closeGripper();
                }
                break;
            case 8:
                //raise arm slightly to avoid getting caught on wall
                if(getStepTime() > delays.grab_post){
                    viperSlide.rotHorPost();
                }
                break;
            case 9:
                //rotate to hang position
                //extend to hang position
                //drive to bar
                if(viperSlide.rotHorPost()){
                    viperSlide.rotHang();
                    viperSlide.extPreHang();
                    follower.followPath(score2, true);
                    setPathState(-1);
                }
                break;
            case 10:
                //retract arm to hang specimen 2
                if (!follower.isBusy() && viperSlide.rotHang() && viperSlide.extPreHang()) {
                    viperSlide.extPostHang();
                }
            case 11:
                //release specimen 2
                if(viperSlide.extPostHang() && getStepTime() > delays.score_pre){
                    viperSlide.openGripper();
                    setPathState(-1);
                }
                break;
            case 12:
                //slightly raise arm away from bar
                if(getStepTime() > delays.score_post){
                    if(viperSlide.rotAwayFromBar()) {
                        setPathState(-1);
                    }
                }
                break;
            case 13:
                //retract arm
                //lower arm to grabbing position
                //drive to OZ
                viperSlide.extHome();
                viperSlide.rotHorPre();
                follower.followPath(pick3, false);
                if(!follower.isBusy() && viperSlide.extHome() && viperSlide.rotHorPre()){
                    setPathState(-1);
                }
                break;
            case 14:
                //extend to grab specimen 3
                if(getStepTime() > delays.grab_pre){
                    viperSlide.extHor();
                }
                break;
            case 15:
                //grab specimen 3
                if(viperSlide.extHor()){
                    viperSlide.closeGripper();
                }
                break;
            case 16:
                //raise arm slightly to avoid getting caught on wall
                if(getStepTime() > delays.grab_post){
                    viperSlide.rotHorPost();
                }
                break;
            case 17:
                //rotate to hang position
                //extend to hang position
                //drive to bar
                if(viperSlide.rotHorPost()){
                    viperSlide.rotHang();
                    viperSlide.extPreHang();
                    follower.followPath(score3, true);
                    setPathState(-1);
                }
            case 18:
                //retract arm to hang specimen 3
                if(viperSlide.extPostHang() && getStepTime() > delays.score_pre){
                    viperSlide.openGripper();
                    setPathState(-1);
                }
                break;
            case 19:
                //release specimen 3
                if(viperSlide.extPostHang() && getStepTime() > delays.score_pre){
                    viperSlide.openGripper();
                    setPathState(-1);
                }
                break;
            case 20:
                //slightly raise arm away from bar
                if(getStepTime() > delays.score_post){
                    if(viperSlide.rotAwayFromBar()) {
                        setPathState(-1);
                    }
                }
            case 21:
                //fully retract arm
                //lower arm to bottom position
                //park
                viperSlide.rotHorPre();
                viperSlide.extHome();
                follower.followPath(park);

        }
    }
    private void setPathState(int pState){
        stepStart = System.nanoTime();
        pathState = pState;
    }
    @Override
    public void init() {

    }

    @Override
    public void loop() {

    }
}
