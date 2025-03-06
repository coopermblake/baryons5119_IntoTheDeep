package org.firstinspires.ftc.teamcode.opmodes.auto;

import static org.firstinspires.ftc.teamcode.opmodes.auto.PPB.*;

import com.pedropathing.follower.Follower;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.core.ViperSlide;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;

@Autonomous(name = "pp basket", group = "Competition Opmodes")
public class PP_Basket extends OpMode {
    private Follower follower;
    ViperSlide viperSlide;

    private PathChain score1, pickup2, score2, pickup3, score3;

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
        score1 = follower.pathBuilder()
                .addPath(new BezierCurve(makePoint(start.class), makePoint(score1_c1.class), makePoint(score1_adj.class)))
                .setLinearHeadingInterpolation(Math.toRadians(start.h), Math.toRadians(score1_adj.h))
                .build();
        pickup2 = follower.pathBuilder()
                .addPath(new BezierLine(makePoint(basket_r.class), makePoint(pick_2_adj.class)))
                .setLinearHeadingInterpolation(Math.toRadians(basket_r.h), Math.toRadians(pick_2_adj.h))
                .build();
        score2 = follower.pathBuilder()
                .addPath(new BezierLine(makePoint(pick_2_r.class), makePoint(score2_adj.class)))
                .setLinearHeadingInterpolation(Math.toRadians(pick_2_r.h), Math.toRadians(score2_adj.h))
                .build();
        pickup3 = follower.pathBuilder()
                .addPath(new BezierLine(makePoint(basket_r.class), makePoint(pick_3_adj.class)))
                .setLinearHeadingInterpolation(Math.toRadians(basket_r.h), Math.toRadians(pick_3_adj.h))
                .build();
        score3 = follower.pathBuilder()
                .addPath(new BezierLine(makePoint(pick_3_r.class), makePoint(score_3_adj.class)))
                .setLinearHeadingInterpolation(Math.toRadians(pick_3_r.h), Math.toRadians(score_3_adj.h))
                .build();
    }

    public void autonomousPathUpdate(){
        switch(pathState){
            case 0:
                pathStart = System.nanoTime();
                stepStart = System.nanoTime();
                setPathState(1);
                break;
            case 1:
                //drive to basket
                viperSlide.rotBasket();
                follower.followPath(score1, true);
                setPathState(2);
                break;
            case 2:
                //extend up to basket
                if(!follower.isBusy() && viperSlide.rotBasket()){
                    viperSlide.extBasket();
                    setPathState(3);
                }
                break;
            case 3:
                //release sample
                if(viperSlide.extBasket() && getStepTime() > delays.release_pre){
                    viperSlide.openGripper();
                    setPathState(4);
                }
                break;
            case 4:
                //rotate slightly back to avoid getting caught on basket
                if(getStepTime() > delays.release_post){
                    viperSlide.basketRotRetract();
                    setPathState(5);
                }
                break;
            case 5:
                //retract slide
                if(viperSlide.basketRotRetract()){
                    viperSlide.extHome();
                    setPathState(6);
                }
                break;
            case 6:
                //lower slide and drive to pickup second sample
                if(viperSlide.extHome()){
                    viperSlide.rotPickPre();
                    follower.followPath(pickup2, true);
                    setPathState(7);
                }
                break;
            case 7:
                //extend slide to pickup position
                if(!follower.isBusy() && viperSlide.rotPickPre()){
                    viperSlide.extPick();
                    setPathState(8);
                }
                break;
            case 8:
                //rotate down onto sample 2
                if(viperSlide.extPick()){
                    viperSlide.rotPickPost();
                    setPathState(9);
                }
                break;
            case 9:
                //grab sample
                if(viperSlide.rotPickPost()){
                    viperSlide.closeGripper();
                    setPathState(10);
                }
                break;
            case 10:
                //retract and rotate up and drive to basket
                if(getStepTime() > delays.pick){
                    viperSlide.rotBasket();
                    viperSlide.extHome();
                    if(getStepTime() > delays.pick + delays.score_move){
                        follower.followPath(score2, true);
                        setPathState(11);
                    }
                }
                break;
            case 11:
                //extend to basket
                if (!follower.isBusy() && viperSlide.rotBasket()) {
                    viperSlide.extBasket();
                    setPathState(12);
                }
                break;
            case 12:
                //release sample
                if(getStepTime() > delays.release_pre && viperSlide.extBasket()){
                    viperSlide.openGripper();
                    setPathState(13);
                }
                break;
            case 13:
                //rotate slightly back to avoid getting caught on basket
                if(getStepTime() > delays.release_post){
                    viperSlide.basketRotRetract();
                    setPathState(14);
                }
                break;
            case 14:
                //retract slide
                if(viperSlide.basketRotRetract()){
                    viperSlide.extHome();
                    setPathState(15);
                }
                break;
            case 15:
                //lower slide and drive to pickup second sample
                if(viperSlide.extHome()){
                    viperSlide.rotPickPre();
                    follower.followPath(pickup3, true);
                    setPathState(16);
                }
                break;
            case 16:
                //extend slide to pickup position
                if(!follower.isBusy() && viperSlide.rotPickPre()){
                    viperSlide.extPick();
                    setPathState(17);
                }
                break;
            case 17:
                //rotate down onto sample 2
                if(viperSlide.extPick()){
                    viperSlide.rotPickPost();
                    setPathState(18);
                }
                break;
            case 18:
                //grab sample
                if(viperSlide.rotPickPost()){
                    viperSlide.closeGripper();
                    setPathState(19);
                }
                break;
            case 19:
                //retract and rotate up and drive to basket
                if(getStepTime() > delays.pick) {
                    viperSlide.rotBasket();
                    viperSlide.extHome();
                    if (getStepTime() > delays.pick + delays.score_move){
                        follower.followPath(score3, true);
                        setPathState(20);
                    }
                }
                break;
            case 20:
                //extend to basket
                if (!follower.isBusy() && viperSlide.rotBasket()) {
                    viperSlide.extBasket();
                    setPathState(21);
                }
                break;
            case 21:
                //release sample
                if(getStepTime() > delays.release_pre && viperSlide.extBasket()){
                    viperSlide.openGripper();
                    setPathState(-1);
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
        follower.setStartingPose(makePose(start.class));
        buildPaths();
        viperSlide = new ViperSlide(hardwareMap);
        viperSlide.closeGripper();
        viperSlide.rotateGripperToCenter();
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
