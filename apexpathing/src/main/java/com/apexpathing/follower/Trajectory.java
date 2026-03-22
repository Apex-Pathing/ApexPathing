package com.apexpathing.follower;

/**
 * Trajectory interface for the CustomDrive base class.
 * @author Krish Joshi - 26192 Heatwaves
 */
public interface Trajectory {
    /**
     * Samples the trajectory at a given time.
     * @param time The time in seconds.
     * @return The trajectory sample at the given time.
     */
    TrajectorySample sample(double time);

    /**
     * Gets the total duration of the trajectory.
     * @return The duration in seconds.
     */
    double duration();
}
