package org.firstinspires.ftc.teamcode.opmodes.auto;

import static org.firstinspires.ftc.teamcode.opmodes.auto.PPP.*;

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

@Autonomous(name = "pp push 3", group = "Competition Opmodes")
public class PP_Push3 extends OpMode {
    //TODO: make robot finish rotating, then drive straight forward when hanging specimen
    //TODO: remove time based delay for retracting arm when scoring

    private Follower follower;
    ViperSlide viperSlide;
    private PathChain score1, push11, push12, push13, push1, score2, pick3, score3, park;
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
                .setPathEndTimeoutConstraint(delays.score1_timeout)
                .build();
        push1 = follower.pathBuilder()
                .addPath(new BezierCurve(makePoint(score1_r.class), makePoint(nextToBar_c1.class), makePoint(nextToBar_adj.class)))
                .setLinearHeadingInterpolation(Math.toRadians(score1_r.h), Math.toRadians(nextToBar_adj.h))
                .setPathEndTimeoutConstraint(5)
                .addPath(new BezierCurve(makePoint(nextToBar_r.class), makePoint(push1_c1.class), makePoint(push1_adj.class)))
                .setConstantHeadingInterpolation(Math.toRadians(nextToBar_r.h))
                .addPath(new BezierLine(makePoint(push1_r.class), makePoint(pickup2_adj.class)))
                .setConstantHeadingInterpolation(Math.toRadians(push1_r.h))
                .build();
        score2 = follower.pathBuilder()
                .addPath(new BezierCurve(makePoint(pickup2_r.class), makePoint(score2_c1.class), makePoint(score2_a.class)))
                .setLinearHeadingInterpolation(Math.toRadians(pickup2_r.h), Math.toRadians(score2_a.h))
                .addPath(new BezierLine(makePoint(score2_a.class), makePoint(score2_adj.class)))
                .setConstantHeadingInterpolation(Math.toRadians(score2_a.h))
                .setZeroPowerAccelerationMultiplier(zpams.score2)
                .build();
        pick3 = follower.pathBuilder()
                .addPath(new BezierCurve(makePoint(score2_r.class), makePoint(pickup3_c1.class), makePoint(pickup3_adj.class)))
                .setLinearHeadingInterpolation(Math.toRadians(score2_r.h), Math.toRadians(pickup3_adj.h))
                .build();
        score3 = follower.pathBuilder()
                .addPath(new BezierCurve(makePoint(pickup3_r.class), makePoint(score3_c1.class), makePoint(score3_a.class)))
                .setLinearHeadingInterpolation(Math.toRadians(pickup3_r.h), Math.toRadians(score3_a.h))
                .addPath(new BezierLine(makePoint(score3_a.class), makePoint(score3_adj.class)))
                .setConstantHeadingInterpolation(Math.toRadians(score3_a.h))
                .setZeroPowerAccelerationMultiplier(zpams.score3)
                .build();
        park = follower.pathBuilder()
                .addPath(new BezierCurve(makePoint(score3_r.class), makePoint(park_c1.class), makePoint(park_r.class)))
                .setConstantHeadingInterpolation(Math.toRadians(score3_r.h))
                .build();


    }

    private void autonomousPathUpdate(){
        switch(pathState){
            case 0:
                //score_raise arm and extend
                viperSlide.rotHang();
                viperSlide.extPreHang();
                setPathState(1);
                break;

            case 1:
                //drive to bar
                if(getStepTime() > delays.start_raise){
                    follower.followPath(score1, true);
                    setPathState(2);
                }
                break;
            case 2:
                //retract arm to score first specimen
                if(!follower.isBusy() && viperSlide.rotHang() && viperSlide.extPreHang() && getStepTime() > delays.score_raise){
                    viperSlide.extPostHang();
                    viperSlide.rotLock();
                    setPathState(3);
                }
                break;
            case 3:
                //release sample
                if(viperSlide.extPostHang() && getStepTime() > delays.score_pre){
                    viperSlide.openGripper();
                    setPathState(4);
                }
                break;
            case 4:
                //slightly score_raise arm away from bar

                if(getStepTime() > delays.score_post){
                    if(viperSlide.rotAwayFromBar()) {
                        setPathState(5);
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
                setPathState(6);
                break;
            case 6:
                if(!follower.isBusy() && viperSlide.extHome() && viperSlide.rotHorPre()){
                    setPathState(7);
                }
                break;
            case 7:
                //extend to grab
                if(getStepTime() > delays.grab_pre){
                    viperSlide.extHor();
                    setPathState(8);
                }
                break;
            case 8:
                //grab specimen
                if(viperSlide.extHor()){
                    viperSlide.closeGripper();
                    setPathState(9);
                }
                break;
            case 9:
                //score_raise arm slightly to avoid getting caught on wall
                if(getStepTime() > delays.grab_post){
                    viperSlide.rotHorPost();
                    setPathState(10);
                }
                break;
            case 10:
                //rotate to hang position
                //extend to hang position
                //drive to bar
                viperSlide.closeGripper();
                if(viperSlide.rotHorPost()){
                    viperSlide.rotHang();
                    viperSlide.extPreHang();
                    follower.followPath(score2, true);
                    setPathState(11);
                }
                break;
            case 11:
                //retract arm to hang specimen 2
                viperSlide.closeGripper();
                if (!follower.isBusy() && viperSlide.rotHang() && viperSlide.extPreHang() && getStepTime() > delays.score_raise) {
                    viperSlide.extPostHang();
                    viperSlide.rotLock();
                    setPathState(12);
                }
                break;
            case 12:
                //release specimen 2
                if(viperSlide.extPostHang() && viperSlide.rotLock() && getStepTime() > delays.score_pre){
                    viperSlide.openGripper();
                    setPathState(13);
                }
                break;
            case 13:
                //slightly score_raise arm away from bar
                if(getStepTime() > delays.score_post){
                    if(viperSlide.rotAwayFromBar()) {
                        setPathState(14);
                    }
                }
                break;
            case 14:
                //retract arm
                //lower arm to grabbing position
                //drive to OZ
                viperSlide.extHome();
                viperSlide.rotHorPre();
                follower.followPath(pick3, false);
                setPathState(15);
                break;
            case 15:
                if(!follower.isBusy() && viperSlide.extHome() && viperSlide.rotHorPre()){
                    setPathState(16);
                }
                break;
            case 16:
                //extend to grab specimen 3
                if(getStepTime() > delays.grab_pre){
                    viperSlide.extHor();
                    setPathState(17);
                }
                break;
            case 17:
                //grab specimen 3
                if(viperSlide.extHor()){
                    viperSlide.closeGripper();
                    setPathState(18);
                }
                break;
            case 18:
                //score_raise arm slightly to avoid getting caught on wall
                viperSlide.closeGripper();
                if(getStepTime() > delays.grab_post){
                    viperSlide.rotHorPost();
                    setPathState(19);
                }
                break;
            case 19:
                //rotate to hang position
                //extend to hang position
                //drive to bar
                viperSlide.closeGripper();
                if(viperSlide.rotHorPost()){
                    viperSlide.rotHang();
                    viperSlide.extPreHang();
                    follower.followPath(score3, true);
                    setPathState(20);
                }
                break;
            case 20:
                //retract arm to hang specimen 3
                viperSlide.closeGripper();
                if (!follower.isBusy() && viperSlide.rotHang() && viperSlide.extPreHang() && getStepTime() > delays.score_raise) {
                    viperSlide.extPostHang();
                    viperSlide.rotLock();
                    setPathState(21);
                }
                break;
            case 21:
                //release specimen 3
                if(viperSlide.extPostHang() && viperSlide.rotLock() && getStepTime() > delays.score_pre){
                    viperSlide.openGripper();
                    setPathState(22);
                }
                break;
            case 22:
                //slightly score_raise arm away from bar
                if(getStepTime() > delays.score_post){
                    if(viperSlide.rotAwayFromBar()) {
                        setPathState(23);
                    }
                }
                break;
            case 23:
                //fully retract arm
                //lower arm to bottom position
                //park
                viperSlide.rotHorPre();
                viperSlide.extHome();
                follower.followPath(park);
                setPathState(-1);
                break;

        }
    }
    private void setPathState(int pState){
        stepStart = System.nanoTime();
        pathState = pState;
    }
    @Override
    public void init() {
        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(PPP.makePose(PPP.start.class));
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
