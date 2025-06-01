
package org.firstinspires.ftc.teamcode.pedroPathing.examples;

import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathBuilder;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;

public class GeneratedPaths {

  public static PathBuilder builder = new PathBuilder();

  public static PathChain line1 = builder
    .addPath(
      new BezierLine(
        new Point(8.000, 63.000, Point.CARTESIAN),
        new Point(36.000, 63.000, Point.CARTESIAN)
      )
    )
    .setConstantHeadingInterpolation(Math.toRadians(0))
    .build();

  public static PathChain line2 = builder
    .addPath(
      new BezierCurve(
        new Point(36.000, 63.000, Point.CARTESIAN),
        new Point(35.000, 39.000, Point.CARTESIAN),
        new Point(36.000, 38.000, Point.CARTESIAN)
      )
    )
    .setConstantHeadingInterpolation(Math.toRadians(0))
    .build();

  public static PathChain line3 = builder
    .addPath(
      new BezierLine(
        new Point(36.000, 38.000, Point.CARTESIAN),
        new Point(28.000, 36.000, Point.CARTESIAN)
      )
    )
    .setLinearHeadingInterpolation(Math.toRadians(-15), Math.toRadians(225))
    .build();

  public static PathChain line4 = builder
    .addPath(
      new BezierLine(
        new Point(28.000, 36.000, Point.CARTESIAN),
        new Point(36.000, 30.000, Point.CARTESIAN)
      )
    )
    .setLinearHeadingInterpolation(Math.toRadians(225), Math.toRadians(330))
    .build();

  public static PathChain line5 = builder
    .addPath(
      new BezierLine(
        new Point(36.000, 30.000, Point.CARTESIAN),
        new Point(28.000, 30.000, Point.CARTESIAN)
      )
    )
    .setLinearHeadingInterpolation(Math.toRadians(330), Math.toRadians(225))
    .build();

  public static PathChain line6 = builder
    .addPath(
      new BezierLine(
        new Point(28.000, 30.000, Point.CARTESIAN),
        new Point(42.000, 22.000, Point.CARTESIAN)
      )
    )
    .setLinearHeadingInterpolation(Math.toRadians(225), Math.toRadians(280))
    .build();

  public static PathChain line7 = builder
    .addPath(
      new BezierLine(
        new Point(42.000, 22.000, Point.CARTESIAN),
        new Point(28.000, 26.000, Point.CARTESIAN)
      )
    )
    .setLinearHeadingInterpolation(Math.toRadians(280), Math.toRadians(180))
    .build();
}
