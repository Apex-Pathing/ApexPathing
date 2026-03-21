package com.apexpathing.kinematics;

/**
 * Interface to allow HolonomicTrajectoryFollower to output a ChassisSpeeds object
 * which is then passed to either MecanumKinematics or SwerveKinematics.
 */
public interface KinematicsSwitcher {
    /**
     * @param chassisSpeeds Target chassis speeds (vx, vy, omega).
     * @return Output from the kinematics model (e.g., wheel speeds or module states).
     */
    Object calculate(ChassisSpeeds chassisSpeeds);
}
