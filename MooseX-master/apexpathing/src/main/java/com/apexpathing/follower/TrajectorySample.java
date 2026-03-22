package com.apexpathing.follower;


import com.apexpathing.util.math.Pose;

/**
 * A sample of a trajectory at a given time or distance.
 * Contains target pose, velocity, and acceleration.
 */
public class TrajectorySample {
    public final Pose pose;
    public final Pose velocity;
    public final Pose acceleration;

    /**
     * @param pose Target pose (x, y, theta)
     * @param velocity Target velocity (vx, vy, vtheta)
     * @param acceleration Target acceleration (ax, ay, atheta)
     */
    public TrajectorySample(Pose pose, Pose velocity, Pose acceleration) {
        this.pose = pose;
        this.velocity = velocity;
        this.acceleration = acceleration;
    }

    public TrajectorySample(double x, double y, double theta, double vx, double vy, double vtheta, double ax, double ay, double atheta) {
        this(new Pose(x, y, theta), new Pose(vx, vy, vtheta), new Pose(ax, ay, atheta));
    }
}
