package org.firstinspires.ftc.teamcode.opmodes.auto;

import static org.firstinspires.ftc.teamcode.opmodes.auto.PP45.makePoint;
import static org.firstinspires.ftc.teamcode.opmodes.auto.PP45.makePose;

import com.pedropathing.follower.Follower;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.opmodes.auto.PP45.*;

import org.firstinspires.ftc.teamcode.core.ViperSlide;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;

@Autonomous(name = "pp push 45", group = "Competition Opmodes")
public class PP_Push45 extends OpMode {
    private Follower follower;
    ViperSlide viperSlide;
    private PathChain score1, push, score2, pick3, score3, pick4, score4, pick5, score5;
    private int pathState;
    private long pathStart;
    private long stepStart;

    private double getPathTime(){
        return (System.nanoTime() - pathStart)/1_000_000_000.0;
    }
    private double getStepTime(){
        return (System.nanoTime() - stepStart)/1_000_000_000.0;
    }

    private void setPathState(int pState){
        stepStart = System.nanoTime();
        pathState = pState;
    }

    public void buildPaths(){
        score1 = follower.pathBuilder()
                .addPath(new BezierLine(makePoint(start.class), makePoint(score1_adj.class)))
                .setConstantHeadingInterpolation(Math.toRadians(start.h))
                .setZeroPowerAccelerationMultiplier(zpams.score1)
                .setPathEndTimeoutConstraint(timeouts.score1)
                .build();

        push = follower.pathBuilder()
                .addPath(new BezierCurve(makePoint(score1_r.class), makePoint(nextToBar_c1.class), makePoint(nextToBar_adj.class)))
                .setLinearHeadingInterpolation(Math.toRadians(score1_r.h), Math.toRadians(nextToBar_adj.h))
                .setPathEndTimeoutConstraint(5)

                .addPath(new BezierCurve(makePoint(nextToBar_r.class), makePoint(allign2_c1.class), makePoint(allign2_adj.class)))
                .setConstantHeadingInterpolation(Math.toRadians(nextToBar_r.h))
                .addPath(new BezierLine(makePoint(allign2_r.class), makePoint(push2_adj.class)))
                .setConstantHeadingInterpolation(Math.toRadians(allign2_r.h))

                .addPath(new BezierCurve(makePoint(push2_r.class), makePoint(allign3_c1.class), makePoint(allign3_adj.class)))
                .setConstantHeadingInterpolation(Math.toRadians(push2_r.h))
                .addPath(new BezierLine(makePoint(allign3_r.class), makePoint(push3_adj.class)))
                .setConstantHeadingInterpolation(Math.toRadians(allign3_r.h))

                .build();
        
        score2 = follower.pathBuilder()
                .addPath(new BezierLine(makePoint(push3_r.class), makePoint(score2_a.class)))
                .setLinearHeadingInterpolation(Math.toRadians(push3_r.h), Math.toRadians(score2_a.h))
                .addPath(new BezierLine(makePoint(score2_a.class), makePoint(score2_adj.class)))
                .setConstantHeadingInterpolation(Math.toRadians(score2_a.h))
                .setZeroPowerAccelerationMultiplier(zpams.score2)
                .setPathEndTimeoutConstraint(timeouts.score2)
                .build();

        pick3 = follower.pathBuilder()
                .addPath(new BezierCurve(makePoint(score2_r.class), makePoint(pickup3_c1.class), makePoint(pickup3.class)))
                .setLinearHeadingInterpolation(Math.toRadians(score2_r.h), Math.toRadians(pickup3.h))
                .setZeroPowerAccelerationMultiplier(zpams.pick3)
                .build();

        score3 = follower.pathBuilder()
                .addPath(new BezierLine(makePoint(pickup_r.class), makePoint(score3_a.class)))
                .setLinearHeadingInterpolation(Math.toRadians(pickup_r.h), Math.toRadians(score3_a.h))
                .addPath(new BezierLine(makePoint(score3_a.class), makePoint(score3_adj.class)))
                .setConstantHeadingInterpolation(Math.toRadians(score3_a.h))
                .setZeroPowerAccelerationMultiplier(zpams.score3)
                .setPathEndTimeoutConstraint(timeouts.score3)
                .build();

        pick4 = follower.pathBuilder()
                .addPath(new BezierCurve(makePoint(score3_r.class), makePoint(pickup4_c1.class), makePoint(pickup4.class)))
                .setLinearHeadingInterpolation(Math.toRadians(score3_r.h), Math.toRadians(pickup4.h))
                .setZeroPowerAccelerationMultiplier(zpams.pick4)
                .build();

        score4 = follower.pathBuilder()
                .addPath(new BezierLine(makePoint(pickup_r.class), makePoint(score4_a.class)))
                .setLinearHeadingInterpolation(Math.toRadians(pickup_r.h), Math.toRadians(score4_a.h))
                .addPath(new BezierLine(makePoint(score4_a.class), makePoint(score4_adj.class)))
                .setConstantHeadingInterpolation(Math.toRadians(score4_a.h))
                .setZeroPowerAccelerationMultiplier(zpams.score4)
                .setPathEndTimeoutConstraint(timeouts.score4)
                .build();

        pick5 = follower.pathBuilder()
                .addPath(new BezierCurve(makePoint(score4_r.class), makePoint(pickup5_c1.class), makePoint(pickup5.class)))
                .setLinearHeadingInterpolation(Math.toRadians(score4_r.h), Math.toRadians(pickup5.h))
                .setZeroPowerAccelerationMultiplier(zpams.pick5)
                .build();

        score5 = follower.pathBuilder()
                .addPath(new BezierLine(makePoint(pickup_r.class), makePoint(score5_a.class)))
                .setLinearHeadingInterpolation(Math.toRadians(pickup_r.h), Math.toRadians(score5_a.h))
                .addPath(new BezierLine(makePoint(score5_a.class), makePoint(score5_adj.class)))
                .setConstantHeadingInterpolation(Math.toRadians(score5_a.h))
                .setZeroPowerAccelerationMultiplier(zpams.score5)
                .setPathEndTimeoutConstraint(timeouts.score5)
                .build();

    }
    public void autonomousPathUpdate(){
        switch(pathState){
            case 0:
                //raise and extend arm
                viperSlide.rotHang();
                viperSlide.extPreHang();
                setPathState(1);
                break;
            case 1:
                //after slight delay, move forward to bar
                if(getStepTime() > delays.start_raise){
                    follower.followPath(score1, false);
                    setPathState(2);
                }
                break;
            case 2:
                //after path is done, retract and rotate down to hang
                viperSlide.closeGripper();
                if(!follower.isBusy() && viperSlide.rotHang() && viperSlide.extPreHang()){
                    viperSlide.extPostHang();
                    viperSlide.rotLock();
                    setPathState(3);
                }
                break;
            case 3:
                //after slight delay, open gripper
                if(viperSlide.extPostHang() && getStepTime() > delays.score_pre){
                    viperSlide.openGripper();
                    setPathState(10);
                }
                break;
            //1111111111111111111111111111111111111111111111111111111    
            case 10:
                //after slight delay, rotate arm back away from bar
                if(getStepTime() > delays.score_post){
                    viperSlide.rotAwayFromBar();
                    setPathState(11);
                }
                break;
            case 11:
                //after arm is away from bar, rotate down to horizontal and retract to home and run full pushing path
                if (viperSlide.rotAwayFromBar()) {
                    viperSlide.extHome();
                    viperSlide.rotHorPre();
                    follower.followPath(push);
                    setPathState(12);
                }
                break;
            case 12:
                //check if all is good
                if(!follower.isBusy() && viperSlide.extHome() && viperSlide.rotHorPre()){
                    setPathState(13);
                }
                break;
            case 13:
                //after slight delay, extend arm to grab specimen 2
                if(getStepTime() > delays.grab_pre){
                    viperSlide.extHor();
                    setPathState(20);
                }
                break;
            //2222222222222222222222222222222222222222222222222222222    
            case 20:
                //after arm is extended, grab specimen 2
                if(viperSlide.extHor()){
                    viperSlide.closeGripper();
                    setPathState(21);
                }
                break;
            case 21:
                //after slight delay, slightly raise arm up
                if(getStepTime() > delays.grab_post){
                    viperSlide.rotHorPost();
                    setPathState(22);
                }
                break;
            case 22:
                //after arm is raised, rotate arm up, extend, and drive to bar
                viperSlide.closeGripper();
                if(viperSlide.rotHorPost()){
                    viperSlide.rotHang();
                    viperSlide.extPreHang();
                    follower.followPath(score2);
                    setPathState(23);
                }
            case 23:
                //after path is done, retract and rotate down to hang
                viperSlide.closeGripper();
                if(!follower.isBusy() && viperSlide.rotHang() && viperSlide.extPreHang()){
                    viperSlide.extPostHang();
                    viperSlide.rotLock();
                    setPathState(24);
                }
                break;
            case 24:
                //after slight delay, open gripper
                if(viperSlide.extPostHang() && getStepTime() > delays.score_pre){
                    viperSlide.openGripper();
                    setPathState(25);
                }
                break;
            case 25:
                //after slight delay, rotate arm back away from bar
                if(getStepTime() > delays.score_post){
                    viperSlide.rotAwayFromBar();
                    setPathState(26);
                }
                break;
            case 26:
                //after slight delay, rotate down to horizontal and retract to home drive to OZ
                if (viperSlide.rotAwayFromBar()) {
                    viperSlide.extHome();
                    viperSlide.rotHorPre();
                    follower.followPath(pick3);
                    setPathState(27);
                }
                break;
            case 27:
                //check if all is good
                if(!follower.isBusy() && viperSlide.extHome() && viperSlide.rotHorPre()){
                    setPathState(28);
                }
                break;
            case 28:
                //after slight delay, extend arm to grab specimen 3
                if(getStepTime() > delays.grab_pre){
                    viperSlide.extHor();
                    setPathState(30);
                }
                break;
            
            //3333333333333333333333333333333333333333333333333333333
            case 30:
                //after arm is extended, grab specimen 3
                if(viperSlide.extHor()){
                    viperSlide.closeGripper();
                    setPathState(31);
                }
                break;
            case 31:
                //after slight delay, slightly raise arm up
                if(getStepTime() > delays.grab_post){
                    viperSlide.rotHorPost();
                    setPathState(32);
                }
                break;
            case 32:
                //after arm is raised, rotate arm up, extend, and drive to bar
                viperSlide.closeGripper();
                if(viperSlide.rotHorPost()){
                    viperSlide.rotHang();
                    viperSlide.extPreHang();
                    follower.followPath(score3);
                    setPathState(33);
                }
            case 33:
                //after path is done, retract and rotate down to hang
                viperSlide.closeGripper();
                if(!follower.isBusy() && viperSlide.rotHang() && viperSlide.extPreHang()){
                    viperSlide.extPostHang();
                    viperSlide.rotLock();
                    setPathState(34);
                }
                break;
            case 34:
                //after slight delay, open gripper
                if(viperSlide.extPostHang() && getStepTime() > delays.score_pre){
                    viperSlide.openGripper();
                    setPathState(35);
                }
                break;
            case 35:
                //after slight delay, rotate arm back away from bar
                if(getStepTime() > delays.score_post){
                    viperSlide.rotAwayFromBar();
                    setPathState(36);
                }
                break;
            case 36:
                //after slight delay, rotate down to horizontal and retract to home drive to OZ
                if (viperSlide.rotAwayFromBar()) {
                    viperSlide.extHome();
                    viperSlide.rotHorPre();
                    follower.followPath(pick4);
                    setPathState(37);
                }
                break;
            case 37:
                //check if all is good
                if(!follower.isBusy() && viperSlide.extHome() && viperSlide.rotHorPre()){
                    setPathState(38);
                }
                break;
            case 38:
                //after slight delay, extend arm to grab specimen 4
                if(getStepTime() > delays.grab_pre){
                    viperSlide.extHor();
                    setPathState(40);
                }
                break;
                
            //44444444444444444444444444444444444444444444444444444444
            case 40:
                //after arm is extended, grab specimen 4
                if(viperSlide.extHor()){
                    viperSlide.closeGripper();
                    setPathState(41);
                }
                break;
            case 41:
                //after slight delay, slightly raise arm up
                if(getStepTime() > delays.grab_post){
                    viperSlide.rotHorPost();
                    setPathState(42);
                }
                break;
            case 42:
                //after arm is raised, rotate arm up, extend, and drive to bar
                viperSlide.closeGripper();
                if(viperSlide.rotHorPost()){
                    viperSlide.rotHang();
                    viperSlide.extPreHang();
                    follower.followPath(score4);
                    setPathState(43);
                }
            case 43:
                //after path is done, retract and rotate down to hang
                viperSlide.closeGripper();
                if(!follower.isBusy() && viperSlide.rotHang() && viperSlide.extPreHang()){
                    viperSlide.extPostHang();
                    viperSlide.rotLock();
                    setPathState(44);
                }
                break;
            case 44:
                //after slight delay, open gripper
                if(viperSlide.extPostHang() && getStepTime() > delays.score_pre){
                    viperSlide.openGripper();
                    setPathState(45);
                }
                break;
            case 45:
                //after slight delay, rotate arm back away from bar
                if(getStepTime() > delays.score_post){
                    viperSlide.rotAwayFromBar();
                    setPathState(46);
                }
                break;
            case 46:
                //after slight delay, rotate down to horizontal and retract to home drive to OZ
                if (viperSlide.rotAwayFromBar()) {
                    viperSlide.extHome();
                    viperSlide.rotHorPre();
                    follower.followPath(pick4);
                    setPathState(47);
                }
                break;
            case 47:
                //check if all is good
                if(!follower.isBusy() && viperSlide.extHome() && viperSlide.rotHorPre()){
                    setPathState(48);
                }
                break;
            case 48:
                //after slight delay, extend arm to grab specimen 5
                if(getStepTime() > delays.grab_pre){
                    viperSlide.extHor();
                    setPathState(-1);
                }
                break;

            //55555555555555555555555555555555555555555555555555555555
//            case 50:
//                //after arm is extended, grab specimen 5
//                if(viperSlide.extHor()){
//                    viperSlide.closeGripper();
//                    setPathState(51);
//                }
//                break;
//            case 51:
//                //after slight delay, slightly raise arm up
//                if(getStepTime() > delays.grab_post){
//                    viperSlide.rotHorPost();
//                    setPathState(52);
//                }
//                break;
//            case 52:
//                //after arm is raised, rotate arm up, extend, and drive to bar
//                viperSlide.closeGripper();
//                if(viperSlide.rotHorPost()){
//                    viperSlide.rotHang();
//                    viperSlide.extPreHang();
//                    follower.followPath(score2);
//                    setPathState(53);
//                }
//            case 53:
//                //after path is done, retract and rotate down to hang
//                viperSlide.closeGripper();
//                if(!follower.isBusy() && viperSlide.rotHang() && viperSlide.extPreHang()){
//                    viperSlide.extPostHang();
//                    viperSlide.rotLock();
//                    setPathState(54);
//                }
//                break;
//            case 54:
//                //after slight delay, open gripper
//                if(viperSlide.extPostHang() && getStepTime() > delays.score_pre){
//                    viperSlide.openGripper();
//                    setPathState(55);
//                }
//                break;
//            case 55:
//                //after slight delay, rotate arm back away from bar
//                if(getStepTime() > delays.score_post){
//                    viperSlide.rotAwayFromBar();
//                    setPathState(56);
//                }
//                break;
//            case 56:
//                //after slight delay, rotate down to horizontal and retract to home drive to OZ
//                if (viperSlide.rotAwayFromBar()) {
//                    viperSlide.extHome();
//                    viperSlide.rotHorPre();
//                    follower.followPath(pick4);
//                    setPathState(-1);
//                }
//                break;

        }
    }
    
    @Override
    public void init(){
        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(makePose(start.class));
        buildPaths();
        viperSlide = new ViperSlide(hardwareMap);
        viperSlide.closeGripper();
    }
    @Override
    public void loop(){
        follower.update();
        autonomousPathUpdate();
        telemetry.addData("Path State", pathState);
        telemetry.addData("Position", follower.getPose().toString());
        telemetry.addData("Path Time", getPathTime());
        telemetry.addData("Step Time", getStepTime());
        telemetry.update();
    }
}
