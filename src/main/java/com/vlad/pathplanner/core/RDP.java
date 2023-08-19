package com.vlad.pathplanner.core;

import java.util.ArrayList;

public class RDP {

    public static double epsilon = 2.5;

    static Vector2D scalarProjection(Vector2D p, Vector2D a, Vector2D b) {
        Vector2D ap = Vector2D.sub(p, a);
        Vector2D ab = Vector2D.sub(b, a);
        ab.normalize();
        ab.mult(ap.dot(ab));
        return Vector2D.add(a, ab);
    }

    static double lineDist(Vector2D c, Vector2D a, Vector2D b) {
        Vector2D norm = scalarProjection(c, a, b);
        return Vector2D.dist(c, norm);
    }

    public static void generateSimplifiedCurve(int startIndex, int endIndex, ArrayList<Vector2D> points,
            ArrayList<Vector2D> rdpPoints) {
        int nextIndex = findFurthest(points, startIndex, endIndex);
        if (nextIndex > 0) {
            if (startIndex != nextIndex) {
                generateSimplifiedCurve(startIndex, nextIndex, points, rdpPoints);
            }
            rdpPoints.add(points.get(nextIndex));
            if (nextIndex != endIndex) {
                generateSimplifiedCurve(nextIndex, endIndex, points, rdpPoints);
            }
        }
    }

    static int findFurthest(ArrayList<Vector2D> points, int a, int b) {
        double maxDistance = -1;
        Vector2D start = points.get(a);
        Vector2D end = points.get(b);
        int furthestIndex = -1;
        for (int i = a + 1; i < b - 1; i++) {
            Vector2D currentPoint = points.get(i);
            double d = lineDist(currentPoint, start, end);
            if (d > maxDistance) {
                maxDistance = d;
                furthestIndex = i;
            }
        }
        if (maxDistance > epsilon) {
            return furthestIndex;
        }
        return -1;
    }
}