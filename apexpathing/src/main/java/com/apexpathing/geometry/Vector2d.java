package com.apexpathing.geometry;

/**
 * A simple 2D vector class for basic math operations.
 */
public class Vector2d {
    public final double x;
    public final double y;

    public Vector2d(double x, double y) {
        this.x = x;
        this.y = y;
    }

    public Vector2d plus(Vector2d other) {
        return new Vector2d(x + other.x, y + other.y);
    }

    public Vector2d minus(Vector2d other) {
        return new Vector2d(x - other.x, y - other.y);
    }

    public Vector2d times(double scalar) {
        return new Vector2d(x * scalar, y * scalar);
    }

    public Vector2d div(double scalar) {
        return new Vector2d(x / scalar, y / scalar);
    }

    public double magnitude() {
        return Math.hypot(x, y);
    }

    public double angle() {
        return Math.atan2(y, x);
    }

    public Vector2d rotate(double angle) {
        double cos = Math.cos(angle);
        double sin = Math.sin(angle);
        return new Vector2d(x * cos - y * sin, x * sin + y * cos);
    }

    public double dot(Vector2d other) {
        return x * other.x + y * other.y;
    }

    public double distanceTo(Vector2d other) {
        return Math.hypot(x - other.x, y - other.y);
    }

    @Override
    public String toString() {
        return String.format("Vector2d(x=%.3f, y=%.3f)", x, y);
    }
}
