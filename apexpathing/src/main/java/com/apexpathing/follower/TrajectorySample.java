package com.apexpathing.follower;

import com.apexpathing.geometry.Pose2d;

/**
 * A sample of a trajectory at a given time or distance.
 * Contains target pose, velocity, and acceleration.
 */
public class TrajectorySample {
    public final Pose2d pose;
    public final Pose2d velocity;
    public final Pose2d acceleration;

    /**
     * @param pose Target pose (x, y, theta)
     * @param velocity Target velocity (vx, vy, vtheta)
     * @param acceleration Target acceleration (ax, ay, atheta)
     */
    public TrajectorySample(Pose2d pose, Pose2d velocity, Pose2d acceleration) {
        this.pose = pose;
        this.velocity = velocity;
        this.acceleration = acceleration;
    }

    public TrajectorySample(double x, double y, double theta, double vx, double vy, double vtheta, double ax, double ay, double atheta) {
        this(new Pose2d(x, y, theta), new Pose2d(vx, vy, vtheta), new Pose2d(ax, ay, atheta));
    }
}
