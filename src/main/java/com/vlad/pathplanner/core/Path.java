package com.vlad.pathplanner.core;

import java.util.ArrayList;
import org.apache.commons.lang3.tuple.ImmutablePair;

import com.vlad.pathplanner.Console;

/**
 * A path contains a list of segments
 * and displacement along the path
 **/
public class Path {

    public static double TANGENT_COEFFICIENT = 0.5;

    private ArrayList<Segment> segments;
    private double length = 0;
    ArrayList<Vector2D> pathPoints = null;

    public Path() {
        segments = new ArrayList<>();
    }

    public Path(ArrayList<Vector2D> points) {
        long startTime = System.currentTimeMillis();
        segments = new ArrayList<>();
        if (points.size() < 2)
            return;
        // First pass - calculate the derivative heuristics
        ArrayList<Vector2D> first_derivatives = new ArrayList<>();

        // Calculate the start derivative
        Vector2D start = points.get(0);
        Vector2D next = points.get(1);
        double angle = Math.atan2(next.y - start.y, next.x - start.x);
        double mag = 0.5 * Vector2D.dist(start, next);
        first_derivatives.add(new Vector2D(mag * Math.cos(angle), mag * Math.sin(angle)));

        for (int i = 1; i < points.size() - 1; i++) {
            Vector2D A = points.get(i - 1);
            Vector2D B = points.get(i);
            Vector2D C = points.get(i + 1);
            Vector2D heuristic = firstDerivativeHeuristic(A, B, C);
            first_derivatives.add(heuristic);
        }
        // Calculate the end derivative
        start = points.get(points.size() - 2);
        next = points.get(points.size() - 1);
        angle = Math.atan2(next.y - start.y, next.x - start.x);
        mag = 0.5 * Vector2D.dist(start, next);
        first_derivatives.add(new Vector2D(mag * Math.cos(angle), mag * Math.sin(angle)));

        for (int i = 0; i < points.size() - 1; i++) {
            Vector2D currentPoint = points.get(i);
            Vector2D nextPoint = points.get(i + 1);

            Segment a = new Segment();
            a.calculateXCoeffs(currentPoint.x, first_derivatives.get(i).x, 0, nextPoint.x,
                    first_derivatives.get(i + 1).x, 0.0);
            a.calculateYCoeffs(currentPoint.y, first_derivatives.get(i).y, 0, nextPoint.y,
                    first_derivatives.get(i + 1).y, 0.0);
            addSegment(a);
            double len = a.length();
            length += len;
        }
        long buildTime = System.currentTimeMillis() - startTime;
        // System.out.println("Path built in " + buildTime + " ms.");
        Console.getInstance().addLine("> Path built in " + buildTime + " ms.\n");
    }

    private ImmutablePair<Segment, Double> getCorrectSegment(double disp) {
        // Immediately throw out absurd displacements
        if (!Utilities.inRange(disp, 0, this.getLength(), 0.01)) {
            System.out.println("Incorrect displacement provided");
            System.exit(-1); // bail
        }
        Segment currentSegment = new Segment();
        boolean found = false;
        double running_disp = 0;
        for (Segment segment : segments) {
            if (Utilities.inRange(disp, running_disp, running_disp + segment.length(), 0.01)) {
                currentSegment = segment;
                found = true;
                break;
            }
            running_disp += segment.length();
        }
        if (!found) {
            System.out.println("Cannot find the correct segment.");
            System.exit(-1);
        }
        return ImmutablePair.of(currentSegment, disp - running_disp);
    }

    public Vector2D getPointAtDisplacement(double s) {
        // Find out in which segment we are
        ImmutablePair<Segment, Double> pair = getCorrectSegment(s);
        Segment correctSegment = pair.left;
        double relative_disp = pair.right;
        // Now evaluate the segment at the correct point
        return correctSegment.getPointAtDisplacement(relative_disp);
    }

    public double getCurvatureAtDisplacement(double s) {
        // Find out in which segment we are
        ImmutablePair<Segment, Double> pair = getCorrectSegment(s);
        Segment correctSegment = pair.left;
        double relative_disp = pair.right;
        double t = correctSegment.getParameterAtDisplacement(relative_disp);
        Vector2D rt1 = correctSegment.getFirstDerivAtParameter(t);
        Vector2D rt2 = correctSegment.getSecondDerivAtParameter(t);
        return Vector2D.absCross(rt1, rt2) / (Math.pow(rt1.copy().mag(), 3));
    }

    public double getSlopeAtDisplacement(double s) {
        // Find out in which segment we are
        ImmutablePair<Segment, Double> pair = getCorrectSegment(s);
        Segment correctSegment = pair.left;
        double relative_disp = pair.right;
        double t = correctSegment.getParameterAtDisplacement(relative_disp);
        return correctSegment.getSlopeAtParameter(t);
    }

    public Vector2D getFirstDerivAtDisplacement(double s) {
        // Find out in which segment we are
        ImmutablePair<Segment, Double> pair = getCorrectSegment(s);
        Segment correctSegment = pair.left;
        double relative_disp = pair.right;
        double t = correctSegment.getParameterAtDisplacement(relative_disp);
        Vector2D rt = correctSegment.getFirstDerivAtParameter(t);
        return rt.copy().normalize();
    }

    private Vector2D firstDerivativeHeuristic(Vector2D A, Vector2D B, Vector2D C) {
        double slopeOfPerpendicularToBisector = -1 / Utilities.getSlopeOfAngleBisector(A, B, C);
        double magnitude = 0.5 * Math.min(Vector2D.dist(B, A), Vector2D.dist(B, C));

        // Orient the tangent vector towards the third point. So dumb it actually works
        double b = -slopeOfPerpendicularToBisector * B.x + B.y;

        Vector2D u = (new Vector2D(0, b).sub(B)).normalize();

        Vector2D truePoint = B.copy().add(u.copy().mult(magnitude));
        Vector2D truePointReflected = B.copy().sub(u.copy().mult(magnitude));

        Vector2D directionVector = new Vector2D(1, slopeOfPerpendicularToBisector).normalize();

        Vector2D point = directionVector.copy().mult(magnitude);
        Vector2D pointReflected = directionVector.copy().mult(-magnitude);

        if (Vector2D.dist(C, truePoint) < Vector2D.dist(C, truePointReflected))
            point = pointReflected.copy();

        double alpha = Math.atan2(point.y, point.x);
        return new Vector2D(magnitude * Math.cos(alpha), magnitude * Math.sin(alpha));
    }

    private Vector2D secondDerivativeHeuristic(Vector2D A, Vector2D B, Vector2D C) {
        return null;
    }

    public double getLength() {
        return length;
    }

    // It is up to the user to make sure the continuities are okay
    public void addSegment(Segment s) {
        segments.add(s);
    }

    public void clear() {
        segments.clear();
    }

    public ArrayList<Vector2D> getPoints() {
        if (segments == null || segments.size() == 0 || length == 0)
            return new ArrayList<>();
        if (pathPoints == null) {
            // if it's not already cached, do it know
            pathPoints = new ArrayList<>();
            for (double s = 0.0; s <= length; s += 5) {
                Vector2D point = getPointAtDisplacement(s);
                pathPoints.add(point);
            }
        }
        return pathPoints;
    }
}