package com.vlad.pathplanner.v2;

public class Point2d {
    public double x = 0, y = 0;

    public Point2d(){}
    public Point2d(double x, double y){
        this.x = x;
        this.y = y;
    }

    @Override
    public boolean equals(Object obj) {
        if(obj instanceof Point2d){
            Point2d point = (Point2d) obj;

            if (point.x == x && point.y == y){
                return true;
            } else {
                return false;
            }
        } else {
            return false;
        }
    }
}
