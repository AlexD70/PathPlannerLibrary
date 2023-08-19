package com.vlad.pathplanner.core;

public class Utilities {
    public static double INF = 999999999;

    public static double getSlopeOfAngleBisector(Vector2D p1, Vector2D p2, Vector2D p3) {
        double abx = angleBisectorX(p1, p2, p3);
        double aby = angleBisectorY(p1, p2, p3);
        double x1 = p2.x - INF * (abx - p2.x);
        double y1 = p2.y - INF * (aby - p2.y);
        double x2 = abx + INF * (abx - p2.x);
        double y2 = aby + INF * (aby - p2.y);
        return (y2 - y1) / (x2 - x1);
    }

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

    public static double[] linspace(double min, double max, int points) {  
        double[] d = new double[points];  
        for (int i = 0; i < points; i++){  
            d[i] = min + i * (max - min) / (points - 1);  
        }  
        return d;  
    }  
}