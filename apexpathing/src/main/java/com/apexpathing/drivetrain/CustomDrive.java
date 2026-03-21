package com.apexpathing.drivetrain;

import com.apexpathing.geometry.Pose2d;
import com.apexpathing.localization.Localizer;
import com.apexpathing.follower.HolonomicTrajectoryFollower;
import com.apexpathing.follower.Trajectory;

/**
 * CustomDrive abstract class following the common library structure.
 */
public abstract class CustomDrive {
    protected Localizer localizer;
    protected HolonomicTrajectoryFollower controller;

    /**
     * Translates a global ChassisSpeeds vector into specific motor commands.
     * @param drivePowers The target Pose2d representing (vx, vy, vtheta).
     */
    public abstract void setDrivePowers(Pose2d drivePowers);

    /**
     * Update loop called once per OpMode loop.
     */
    public abstract void update();

    /**
     * Start following a trajectory.
     * @param traj The trajectory to follow.
     */
    public void followTrajectory(Trajectory traj) {
    }
}
