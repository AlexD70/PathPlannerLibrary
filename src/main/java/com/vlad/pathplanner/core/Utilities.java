package com.vlad.pathplanner.core;

public class Utilities {
    public static double INF = 999999999;

    // less complicated now
    public static double getSlopeOfAngleBisector(Vector2D p1, Vector2D p2, Vector2D p3) {
        double slope1 = (p1.y - p2.y) / (p1.x - p2.x);
        double slope2 = (p2.y - p3.y) / (p2.x - p3.x);

        slope1 = Math.atan(slope1);
        slope2 = Math.atan(slope2);

        return Math.tan((slope1 + slope2)/2);
    }

    // what???
    private static double angleBisectorX(Vector2D p1, Vector2D p2, Vector2D p3) {
        double d1 = Vector2D.dist(p1, p2);
        double d3 = Vector2D.dist(p2, p3);
        double x1 = (d1 - 1) / d1 * p2.x + 1 / d1 * p1.x;
        double y1 = (d1 - 1) / d1 * p2.y + 1 / d1 * p1.y;
        double x3 = (d3 - 1) / d3 * p2.x + 1 / d3 * p3.x;
        double y3 = (d3 - 1) / d3 * p2.y + 1 / d3 * p3.y;
        double x2 = (x1 + x3) / 2;
        double y2 = (y1 + y3) / 2;
        double d2 = Vector2D.dist(new Vector2D(p2.x, p2.y), new Vector2D(x2, y2));
        x2 = (d2 - 1) / d2 * p2.x + 1 / d2 * x2;
        return x2;
    }

    // again what???
    private static double angleBisectorY(Vector2D p1, Vector2D p2, Vector2D p3) {
        double d1 = Vector2D.dist(p1, p2);
        double d3 = Vector2D.dist(p2, p3);
        double x1 = (d1 - 1) / d1 * p2.x + 1 / d1 * p1.x;
        double y1 = (d1 - 1) / d1 * p2.y + 1 / d1 * p1.y;
        double x3 = (d3 - 1) / d3 * p2.x + 1 / d3 * p3.x;
        double y3 = (d3 - 1) / d3 * p2.y + 1 / d3 * p3.y;
        double x2 = (x1 + x3) / 2;
        double y2 = (y1 + y3) / 2;
        double d2 = Vector2D.dist(new Vector2D(p2.x, p2.y), new Vector2D(x2, y2));
        y2 = (d2 - 1) / d2 * p2.y + 1 / d2 * y2;
        return y2;
    }

    public static boolean inRange(double x, double a, double b, double eps) {
        return a - eps <= x && x <= b + eps;
    }

    // optimized - now step is computed only once
    public static double[] generateLinear(double min, double max, int points) {
        double step = (max - min) / (points - 1);
        double[] d = new double[points];

        d[0] = min;
        for (int i = 1; i < points - 1; i++){
            d[i] = d[i - 1] + step;
        }
        d[points - 1] = max;

        return d;
    }  
}