package org.firstinspires.ftc.teamcode.pedroPathing.examples;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.*;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;

@Disabled//(name = "test auto")
public class TestAuto extends OpMode {
    private Follower follower;
    private final Pose startPose = new Pose(0,0,Math.toRadians(0));
    private final Pose barPose = new Pose(24,0,Math.toRadians(90));
    private final Pose push1Control1 = new Pose (16, 16, Math.toRadians(135));
    private final Pose push1Control2 = new Pose(28,39,Math.toRadians(180));
    private final Pose push1 = new Pose (44, 54, Math.toRadians(270));
    private Path scorePreload;
    private Path push1chain;

    private int pathState;

    public void buildPaths(){
        scorePreload = new Path(new BezierLine(new Point(startPose), new Point(barPose)));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), barPose.getHeading());

        push1chain = new Path(new BezierCurve(new Point(barPose), new Point(push1Control1),
                new Point(push1Control2), new Point(push1)));
    }
    public void autonomousPathUpdate() {
        switch (pathState){
            case 0:
                follower.followPath(scorePreload);
                setPathState(1);
                break;
            case 1:
                 setPathState(2);
                break;
            case 2:
                setPathState(-1);
                break;
        }

    }

    public void setPathState(int pState){
        pathState = pState;
    }
    @Override
    public void init(){
        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);
        buildPaths();
    }

    @Override
    public void loop(){
        follower.update();
        autonomousPathUpdate();
        telemetry.addData("Path State", pathState);
        telemetry.addData("Position", follower.getPose().toString());
        telemetry.update();

    }
}
