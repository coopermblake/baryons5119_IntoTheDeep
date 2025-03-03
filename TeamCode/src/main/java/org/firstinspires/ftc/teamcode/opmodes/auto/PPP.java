package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.Point;

public class PPP {
    @Config
    public static class delays{
        public static double start_raise = 1;
        public static double score_pre = 1;
        public static double score_post = 1;
        public static double grab_pre = 1;
        public static double grab_post = 1;
    }

    @Config
    public static class start{
        public static double x;
        public static double y;
        public static double h;
    }
    @Config
    public static class score1_adj{
        public static double x;
        public static double y;
        public static double h;
    }
    @Config
    public static class score1_r{
        public static double x;
        public static double y;
        public static double h;
    }
    @Config
    public static class nextToBar_adj{
        public static double x;
        public static double y;
        public static double h;
    }
    @Config
    public static class nextToBar_c1{
        public static double x;
        public static double y;
    }
    @Config
    public static class nextToBar_r{
        public static double x;
        public static double y;
        public static double h;
    }
    @Config
    public static class push1_adj{
        public static double x;
        public static double y;
        public static double h;
    }
    @Config
    public static class push1_c1{
        public static double x;
        public static double y;
    }
    @Config
    public static class push1_r{
        public static double x;
        public static double y;
        public static double h;
    }
    @Config
    public static class pickup2_adj{
        public static double x;
        public static double y;
        public static double h;
    }
    @Config
    public static class pickup2_r{
        public static double x;
        public static double y;
        public static double h;
    }
    @Config
    public static class score2_adj{
        public static double x;
        public static double y;
        public static double h;
    }
    @Config
    public static class score2_c1{}
    @Config
    public static class score2_r{
        public static double x;
        public static double y;
        public static double h;
    }
    @Config
    public static class pickup3_adj{
        public static double x;
        public static double y;
        public static double h;
    }
    @Config
    public static class pickup3_c1{
        public static double x;
        public static double y;
    }
    @Config
    public static class pickup3_r{
        public static double x;
        public static double y;
        public static double h;
    }
    @Config
    public static class score3_adj{
        public static double x;
        public static double y;
        public static double h;
    }
    @Config
    public static class score3_c1{
        public static double x;
        public static double y;
    }
    @Config
    public static class score3_r{
        public static double x;
        public static double y;
        public static double h;
    }
    @Config
    public static class park_adj{
        public static double x;
        public static double y;
        public static double h;
    }
    @Config
    public static class park_c1{
        public static double x;
        public static double y;
    }
    @Config
    public static class park_r{
        public static double x;
        public static double y;
        public static double h;
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
