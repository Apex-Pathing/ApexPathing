package com.apexpathing.geometry;

/**
 * A 2D pose class (x, y, heading).
 */
public class Pose2d {
    public final double x;
    public final double y;
    public final double heading;

    public Pose2d(double x, double y, double heading) {
        this.x = x;
        this.y = y;
        this.heading = heading;
    }

    public Pose2d(Vector2d vec, double heading) {
        this(vec.x, vec.y, heading);
    }

    public Vector2d vec() {
        return new Vector2d(x, y);
    }

    public Pose2d plus(Pose2d other) {
        return new Pose2d(x + other.x, y + other.y, heading + other.heading);
    }

    public Pose2d minus(Pose2d other) {
        return new Pose2d(x - other.x, y - other.y, heading - other.heading);
    }

    @Override
    public String toString() {
        return String.format("Pose2d(x=%.3f, y=%.3f, heading=%.3f)", x, y, heading);
    }
}
