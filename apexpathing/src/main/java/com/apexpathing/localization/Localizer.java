package com.apexpathing.localization;

import com.apexpathing.geometry.Pose2d;

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
    Pose2d getPose();

    /**
     * Returns the current estimated velocity.
     */
    Pose2d getVelocity();

    /**
     * Sets the current pose estimate.
     */
    void setPose(Pose2d pose);
}
