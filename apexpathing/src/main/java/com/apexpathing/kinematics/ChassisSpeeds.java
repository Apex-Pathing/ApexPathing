package com.apexpathing.kinematics;

/**
 * Represents robot-relative velocities.
 */
public class ChassisSpeeds {
    public final double vx;
    public final double vy;
    public final double omega;

    public ChassisSpeeds(double vx, double vy, double omega) {
        this.vx = vx;
        this.vy = vy;
        this.omega = omega;
    }

    @Override
    public String toString() {
        return String.format("ChassisSpeeds(vx=%.3f, vy=%.3f, omega=%.3f)", vx, vy, omega);
    }
}
