package com.vlad.pathplanner.v2;

public class GeometryUtils {

    // returns the bisector line of angle AOB
    public static Line2d buildBisector(Point2d a, Point2d o, Point2d b){
        // find angle vertexes
        Line2d ao = new Line2d(a, o), ob = new Line2d(o, b);

        // find slope of bisector
        double slope = Math.tan( (ao.getAngleWithOX() + ob.getAngleWithOX()) / 2 );

        // build new line
        return new Line2d(o, slope);
    }
}
