package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.Point;

@Config
public class PPPP {
    @Config
    public static class delays{
        public static double start_raise = 1;
        public static double score_pre = 1;
        public static double score_post = 1;
        public static double grab_pre = 1;
        public static double grab_post = 1;
        public static double score_raise = 5;
    }
    @Config
    public static class zpams{
        public static double push2 = 1;
        public static double push3 = 1;
    }

    @Config
    public static class start{
        public static double x = 8;
        public static double y = 63;
        public static double h = 0;
    }
    @Config
    public static class score1_adj{
        public static double x = 36;
        public static double y = 63;
        public static double h = 0;
    }
    @Config
    public static class score1_r{
        public static double x = 36;
        public static double y = 63;
        public static double h = 0;
    }
    @Config
    public static class nextToBar_adj{
        public static double x = 48;
        public static double y = 34.5;
        public static double h = 180;
    }
    @Config
    public static class nextToBar_c1{
        public static double x = 20;
        public static double y = 35;
    }
    @Config
    public static class nextToBar_r{
        public static double x = 48;
        public static double y = 32.5;
        public static double h = 180;
    }
    @Config
    public static class allign2_adj{
        public static double x = 59;
        public static double y = 27.5;
        public static double h = 180;
    }
    @Config
    public static class allign2_c1{
        public static double x = 60;
        public static double y = 33.5;
    }
    @Config
    public static class allign2_r{
        public static double x = 59;
        public static double y = 27.5;
        public static double h = 180;
    }
    @Config
    public static class push2_adj{
        public static double x = 18;
        public static double y = 27.5;
        public static double h = 180;
    }
    @Config
    public static class push2_r{
        public static double x = 18;
        public static double y = 27.5;
        public static double h = 180;
    }
    @Config
    public static class allign3_adj{
        public static double x = 61;
        public static double y = 16;
        public static double h = 0;
    }
    @Config
    public static class allign3_c1{
        public static double x = 64;
        public static double y = 20;
    }
    @Config
    public static class allign3_r{
        public static double x = 61;
        public static double y = 16;
        public static double h = 0;
    }
    @Config
    public static class push3_adj{
        public static double x = 12;
        public static double y = 16;
        public static double h = 0;
    }
    @Config
    public static class push3_r{
        public static double x = 12;
        public static double y = 16;
        public static double h = 0;
    }
    public static class score2_adj{
        public static double x = 36;
        public static double y = 64;
        public static double h = 0;
    }
    @Config
    public static class score2_c1{
        public static double x = 12;
        public static double y = 70;
    }
    @Config
    public static class score2_a{
        public static double x = 30;
        public static double y = 64;
        public static double h = 0;
    }
    @Config
    public static class score2_r{
        public static double x = 36;
        public static double y = 64;
        public static double h = 0;
    }
    @Config
    public static class pickup3_adj{
        public static double x = 12;
        public static double y = 27.5;
        public static double h = 180;
    }
    @Config
    public static class pickup3_c1{
        public static double x = 12;
        public static double y = 70;
    }
    @Config
    public static class pickup3_r{
        public static double x = 12;
        public static double y = 27.5;
        public static double h = 180;
    }
    @Config
    public static class score3_adj{
        public static double x = 36;
        public static double y = 62;
        public static double h = 0;
    }
    @Config
    public static class score3_c1{
        public static double x = 12;
        public static double y = 70;
    }
    @Config
    public static class score3_a{
        public static double x = 30;
        public static double y = 62;
        public static double h = 0;
    }
    @Config
    public static class score3_r{
        public static double x = 36;
        public static double y = 62;
        public static double h = 0;
    }
    @Config
    public static class pickup4_adj{
        public static double x = 12;
        public static double y = 27.5;
        public static double h = 180;
    }
    @Config
    public static class pickup4_c1{
        public static double x = 12;
        public static double y = 70;
    }
    @Config
    public static class pickup4_r{
        public static double x = 12;
        public static double y = 27.5;
        public static double h = 180;
    }
    @Config
    public static class score4_adj{
        public static double x = 36;
        public static double y = 62;
        public static double h = 0;
    }
    @Config
    public static class score4_c1{
        public static double x = 12;
        public static double y = 70;
    }
    @Config
    public static class score4_a{
        public static double x = 30;
        public static double y = 62;
        public static double h = 0;
    }
    @Config
    public static class score4_r{
        public static double x = 36;
        public static double y = 62;
        public static double h = 0;
    }
    @Config
    public static class park_adj{
        public static double x = 12;
        public static double y = 27.5;
        public static double h = 0;
    }
    @Config
    public static class park_c1{
        public static double x = 12;
        public static double y = 70;
    }
    @Config
    public static class park_r{
        public static double x = 12;
        public static double y = 27.5;
        public static double h = 0;
    }

    public static Pose makePose(Class<?> cls) {
        try {
            //Get the field values from the class
            double x = cls.getField("x").getDouble(null);
            double y = cls.getField("y").getDouble(null);
            double h = cls.getField("h").getDouble(null);

            //Create and return a new Pose object
            return new Pose(x, y, Math.toRadians(h));
        } catch (Exception e) {
            e.printStackTrace();
            return null;
        }
    }

    public static Point makePoint(Class<?> cls) {
        try {
            //Get the field values from the class
            double x = cls.getField("x").getDouble(null);
            double y = cls.getField("y").getDouble(null);
            return new Point(x,y);

        } catch (Exception e) {
            throw new RuntimeException(e);
        }
    }
}
