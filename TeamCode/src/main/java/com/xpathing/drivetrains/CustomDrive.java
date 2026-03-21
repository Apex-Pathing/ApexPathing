package com.xpathing.drivetrains;

import org.firstinspires.ftc.teamcode.movement.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.movement.localization.Localizer;
import com.xpathing.main.follower.xpathing.HolonomicTrajectoryFollower;

/**
 * CustomDrive abstract class following the common library structure.
 */
public abstract class CustomDrive {
    protected Localizer localizer;
    protected HolonomicTrajectoryFollower controller; // The Spline Follower

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
        // controller.startFollowing(traj); // Assuming HolonomicTrajectoryFollower has this
    }
}
