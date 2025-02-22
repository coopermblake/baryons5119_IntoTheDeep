package org.firstinspires.ftc.teamcode.opmodes.auto;

import static org.firstinspires.ftc.teamcode.opmodes.auto.PPS.makePoint;
import static org.firstinspires.ftc.teamcode.opmodes.auto.PPS.makePose;

import com.pedropathing.follower.Follower;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.Point;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Autonomous(name = "pp sweep 1", group = "Competition Opmodes")
public class PP_Sweep extends OpMode {
    private Follower follower;

    Point home = new Point(PPS.home.x, PPS.home.y);

    private Path scorePreload, allign1, drag1, allign2, drag2, allign3, drag3, score2, grab3, score3, grab4, score4, grab5, score5;
    private int pathState;

    public void buildPaths(){
        scorePreload = new Path(new BezierLine(makePoint(PPS.home.class), makePoint(PPS.bar_1.class)));
        allign1 = new Path(new BezierCurve(makePoint(PPS.bar_r.class), makePoint(PPS.allign_1_c1.class), makePoint(PPS.allign_1_adj.class)));
        drag1 = new Path(new BezierLine(makePoint(PPS.allign_1_r.class), makePoint(PPS.drag_1_adj.class)));

        allign2 = new Path(new BezierLine(makePoint(PPS.drag_1_r.class), makePoint(PPS.allign_2_adj.class)));
        drag2 = new Path(new BezierLine(makePoint(PPS.allign_2_r.class), makePoint(PPS.drag_2_adj.class)));

        allign3 = new Path(new BezierLine(makePoint(PPS.drag_2_r.class), makePoint(PPS.allign_3_adj.class)));
        drag3 = new Path(new BezierLine(makePoint(PPS.allign_3_r.class), makePoint(PPS.drag_3_adj.class)));

        score2 = new Path(new BezierCurve(makePoint(PPS.drag_3_r.class), makePoint(PPS.hang_c2.class), makePoint(PPS.bar_2.class)));

        grab3 = new Path(new BezierCurve(makePoint(PPS.bar_r.class), makePoint(PPS.grab_c3.class), makePoint(PPS.grab_3.class)));
        score3 = new Path(new BezierCurve(makePoint(PPS.grab_r.class), makePoint(PPS.hang_c3.class), makePoint(PPS.bar_3.class)));

        grab4 = new Path(new BezierCurve(makePoint(PPS.bar_r.class), makePoint(PPS.grab_c4.class), makePoint(PPS.grab_4.class)));
        score4 = new Path(new BezierCurve(makePoint(PPS.grab_r.class), makePoint(PPS.hang_c4.class), makePoint(PPS.bar_4.class)));

        grab5 = new Path(new BezierCurve(makePoint(PPS.bar_r.class), makePoint(PPS.grab_c5.class), makePoint(PPS.grab_5.class)));
        score5 = new Path(new BezierCurve(makePoint(PPS.grab_r.class), makePoint(PPS.hang_c5.class), makePoint(PPS.bar_5.class)));
    }

    @Override
    public void init() {

    }

    @Override
    public void loop() {

    }
}
