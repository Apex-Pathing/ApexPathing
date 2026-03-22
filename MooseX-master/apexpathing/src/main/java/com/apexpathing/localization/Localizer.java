package com.apexpathing.localization;


import com.apexpathing.util.math.Pose;

/**
 * Localizer interface for tracking the robot's pose and velocity.
 */
public interface Localizer {
    /**
     * Updates the localizer's estimate.
     */
    void update();

    /**
     * Returns the current estimated pose.
     */
    Pose getPose();

    /**
     * Returns the current estimated velocity.
     */
    Pose getVelocity();

    /**
     * Sets the current pose estimate.
     */
    void setPose(Pose pose);
}
