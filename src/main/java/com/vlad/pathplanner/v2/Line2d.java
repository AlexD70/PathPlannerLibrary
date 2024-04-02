package com.vlad.pathplanner.v2;

public class Line2d {
    public Point2d x, y;
    public double slope;

    public Line2d(Point2d x, Point2d y){
        if (x.equals(y)){
            throw new RuntimeException("Lines cannot be generated from two identical points");
        }

        this.x = x;
        this.y = y;

        // solve the system:
        // a * xA + b = yA
        // a * xB + b = yB

        slope = (x.y - y.y) / (x.x - y.x);
    }

    public Line2d(Point2d x, double slope){
        this.x = x;
        this.slope = slope;

        // line equation:
        // y - yA = slope * (x - xA)
    }

    public double getLineSlope(){
        return slope;
    }

    public static double getLineSlope(Point2d x, Point2d y){
        return new Line2d(x, y).getLineSlope();
    }

    // returns the angle of this line with the OX axis in RADIANS
    public double getAngleWithOX(){
        return Math.atan(slope);
    }

    public double getAngleWithOY(){
        return Math.PI / 2 - Math.atan(slope);
    }
}
