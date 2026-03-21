package com.apexpathing.kinematics;

/**
 * Represents a swerve module's speed and angle.
 */
public class SwerveModuleState {
    public final double speed;
    public final double angle;

    public SwerveModuleState(double speed, double angle) {
        this.speed = speed;
        this.angle = angle;
    }

    @Override
    public String toString() {
        return String.format("SwerveModuleState(speed=%.3f, angle=%.3f)", speed, angle);
    }
}
