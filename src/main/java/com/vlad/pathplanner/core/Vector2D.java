package com.vlad.pathplanner.core;

import java.io.Serializable;

public class Vector2D implements Serializable {
    public double x;
    public double y;

    public Vector2D() {
        x = 0.0;
        y = 0.0;
    }

    public Vector2D(final double x, final double y) {
        this.x = x;
        this.y = y;
    }

    public Vector2D set(final double x, final double y) {
        this.x = x;
        this.y = y;
        return this;
    }

    public Vector2D copy() {
        return new Vector2D(this.x, this.y);
    }

    public static double dot(final Vector2D v1, final Vector2D v2) {
        return v1.x * v2.x + v1.y * v2.y;
    }

    public double mag() {
        return Math.sqrt(this.x * this.x + this.y * this.y);
    }

    public Vector2D add(final Vector2D v) {
        this.x += v.x;
        this.y += v.y;
        return this;
    }

    public Vector2D add(final double x, final double y) {
        this.x += x;
        this.y += y;
        return this;
    }

    public static Vector2D add(final Vector2D v1, final Vector2D v2) {
        return new Vector2D(v1.x + v2.x, v1.y + v2.y);
    }

    public Vector2D sub(final Vector2D v) {
        this.x -= v.x;
        this.y -= v.y;
        return this;
    }

    public Vector2D sub(final double x, final double y) {
        this.x -= x;
        this.y -= y;
        return this;
    }

    public static Vector2D sub(final Vector2D v1, final Vector2D v2) {
        return new Vector2D(v1.x - v2.x, v1.y - v2.y);
    }

    public Vector2D mult(final double n) {
        this.x *= n;
        this.y *= n;
        return this;
    }

    public Vector2D div(final double n) {
        this.x /= n;
        this.y /= n;
        return this;
    }

    public static double absCross(Vector2D a, Vector2D b) {
        return a.x * b.y - b.x * a.y;
    }

    public static double dist(final Vector2D v1, final Vector2D v2) {
        final double dx = v1.x - v2.x;
        final double dy = v1.y - v2.y;
        return Math.sqrt(dx * dx + dy * dy);
    }

    public Vector2D normalize() {
        final double m = this.mag();
        if (m != 0 && m != 1.0) {
            this.div(m);
        }
        return this;
    }

    public double dot(final Vector2D v) {
        return this.x * v.x + this.y * v.y;
    }
}