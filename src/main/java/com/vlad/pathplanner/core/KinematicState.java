package com.vlad.pathplanner.core;

/**
 * An object that completely (not really, but assuming constant jerk is good enough) describes
 * a kinematic state in time
 **/
public class KinematicState {

    public double x = 0, v = 0, a = 0, j = 0;
    public double t;

    public KinematicState() {

    }

    public KinematicState(double t, double x, double v, double a, double j) {
        this.t = t;
        this.x = x;
        this.v = v;
        this.a = a;
        this.j = j;
    }
}
