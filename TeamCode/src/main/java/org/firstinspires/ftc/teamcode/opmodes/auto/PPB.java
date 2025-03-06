package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.Point;

public class PPB {
    @Config
    public static class delays{
        public static double release_pre = 1;
        public static double release_post = 1;
        public static double pick = 0.5;
        public static double score_move = 1;
    }
    @Config
    public static class start{
        public static double x = 8;
        public static double y = 110;
        public static double h = 0;
    }

    @Config
    public static class basket_r{
        public static double x = 15;
        public static double y = 125;
        public static double h = 135;
    }
    @Config
    public static class score1_c1{
        public static double x = 24;
        public static double y = 120;
    }
    @Config
    public static class score1_adj{
        public static double x = 15;
        public static double y = 125;
        public static double h = 135;
    }
    @Config
    public static class pick_2_adj{
        public static double x = 27;
        public static double y = 121.25;
        public static double h = 0;
    }
    @Config
    public static class pick_2_adj_left{
        public static double x = 27;
        public static double y = 121.25;
        public static double h = 2.5;
    }
    @Config
    public static class pick_2_adj_right{
        public static double x = 27;
        public static double y = 121.25;
        public static double h = -2.5;
    }
    @Config
    public static class  pick_2_r{
        public static double x = 30;
        public static double y = 121.25;
        public static double h = 0;
    }

    @Config
    public static class score2_adj{
        public static double x = 15;
        public static double y = 125;
        public static double h = 135;
    }
    @Config
    public static class pick_3_adj{
        public static double x = 27.5;
        public static double y = 130;
        public static double h = 0;
    }

    @Config
    public static class pick_3_r{
        public static double x = 30;
        public static double y = 132.5;
        public static double h = 0;
    }
    @Config
    public static class score_3_adj{
        public static double x = 15;
        public static double y = 125;
        public static double h = 135;
    }

    @Config
    public static class pick_4_r{
        public static double x = 43.5;
        public static double y = 127.5;
        public static double h = 90;
    }

    public static Pose makePose(Class<?> cls) {
        try {
            // Get the field values from the class
            double x = cls.getField("x").getDouble(null);
            double y = cls.getField("y").getDouble(null);
            double h = cls.getField("h").getDouble(null);

            // Create and return a new Pose object
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
