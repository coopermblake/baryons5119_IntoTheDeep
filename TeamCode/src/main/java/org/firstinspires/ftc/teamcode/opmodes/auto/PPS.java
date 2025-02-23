package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.Point;


public class PPS {

    @Config
    public static class delays{
        public static double grabPreDelay = 0.9;
        public static double grabPostDelay = 0.6;
        public static double grabRaiseMoveDelay = 0.5;
        public static double hangPreDelay = 0.0;
        public static double hangPostDelay = 0.5;
        public static double startDelay = 5.0;
        public static double sweeperDeployDelay = 5.0;
    }
    @Config
    public static class home{
        //starting pose
        public static double x = 8;
        public static double y = 63;
        public static double h = 0;
    }


    //these are ordered by the specimen hung on the bar
    @Config
    public static class bar_r{
        //theoretical bar pose
        public static double x = 36;
        public static double y = 63;
        public static double h = 0;
    }
    @Config
    public static class bar_1{
        //adjusted bar pose for hang 1
        public static double x = bar_r.x;
        public static double y = bar_r.y;
        public static double h = bar_r.h;
    }
    @Config
    public static class bar_2{
        //adjusted bar pose for hang 2
        public static double x = 38;
        public static double y = 63;
        public static double h = 0;
    }
    @Config
    public static class bar_3{
        //adjusted bar pose for hang 3
        public static double x = 38;
        public static double y = 63;
        public static double h = 0;
    }
    @Config
    public static class bar_4{
        //adjusted bar pose for hang 4
        public static double x = 38;
        public static double y = 63;
        public static double h = 0;
    }
    @Config
    public static class bar_5{
        //adjusted bar pose for hang 5
        public static double x = 38;
        public static double y = 63;
        public static double h = 0;
    }


    //these are ordered by the sample pushed
    @Config
    public static class allign_1_r{
        public static double x = 36;
        public static double y = 37.5;
        public static double h = -15;
    }
    @Config
    public static class allign_1_c1{
        public static double x = 35;
        public static double y = 39;
    }
    @Config
    public static class allign_1_adj{
        public static double x = 36;
        public static double y = 37.5;
        public static double h = -15;
    }

    @Config
    public static class drag_1_r{
        public static double x = 38;
        public static double y = 36;
        public static double h = 225;
    }
    @Config
    public static class drag_1_adj{
        public static double x = 38;
        public static double y = 36;
        public static double h = 225;
    }

    @Config
    public static class allign_2_r{
        public static double x = 36;
        public static double y = 30;
        public static double h = 330;
    }
    @Config
    public static class allign_2_adj{
        public static double x = 36;
        public static double y = 30;
        public static double h = 330;
    }

    @Config
    public static class drag_2_r{
        public static double x = 28;
        public static double y = 30;
        public static double h = 225;
    }
    @Config
    public static class drag_2_adj{
        public static double x = 28;
        public static double y = 30;
        public static double h = 225;
    }

    @Config
    public static class allign_3_r{
        public static double x = 42;
        public static double y = 22;
        public static double h = 280;
    }
    @Config
    public static class allign_3_adj{
        public static double x = 42;
        public static double y = 22;
        public static double h = 280;
    }

    @Config
    public static class drag_3_r{
        public static double x = 28;
        public static double y = 26;
        public static double h = 180;
    }
    @Config
    public static class drag_3_adj{
        public static double x = 28;
        public static double y = 26;
        public static double h = 180;
    }

    @Config
    public static class hang_c2{
        public static double x = 25;
        public static double y = 65;
    }
    @Config
    public static class hang_c3{
        public static double x = 25;
        public static double y = 65;
    }
    @Config
    public static class hang_c4{
        public static double x = 25;
        public static double y = 65;
    }
    @Config
    public static class hang_c5{
        public static double x = 25;
        public static double y = 65;
    }

    @Config
    public static class grab_c3{
        public static double x = 25;
        public static double y = 65;
    }
    @Config
    public static class grab_c4{
        public static double x = 25;
        public static double y = 65;
    }
    @Config
    public static class grab_c5{
        public static double x = 25;
        public static double y = 65;
    }

    @Config
    public static class grab_r{
        public static double x = 12;
        public static double y = 32;
        public static double h = 180;
    }
    @Config
    public static class grab_2{
        public static double x = 12;
        public static double y = 32;
        public static double h = 180;
    }
    @Config
    public static class grab_3{
        public static double x = 12;
        public static double y = 32;
        public static double h = 180;
    }
    @Config
    public static class grab_4{
        public static double x = 12;
        public static double y = 32;
        public static double h = 180;
    }
    @Config
    public static class grab_5{
        public static double x = 12;
        public static double y = 32;
        public static double h = 180;
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
