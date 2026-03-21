package com.apexpathing.kinematics;

import com.apexpathing.geometry.Vector2d;

/**
 * Specialized SwerveKinematics class.
 */
public class SwerveKinematics extends Kinematics {
    private final Vector2d[] moduleOffsets;

    /**
     * @param moduleOffsets Offsets of the swerve modules from the robot center (FL, FR, BL, BR).
     */
    public SwerveKinematics(Vector2d[] moduleOffsets) {
        if (moduleOffsets.length != 4) {
            throw new IllegalArgumentException("SwerveKinematics requires 4 module offsets.");
        }
        this.moduleOffsets = moduleOffsets;
    }

    @Override
    public SwerveModuleState[] calculate(ChassisSpeeds chassisSpeeds) {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (int i = 0; i < 4; i++) {
            double vx = chassisSpeeds.vx - chassisSpeeds.omega * moduleOffsets[i].y;
            double vy = chassisSpeeds.vy + chassisSpeeds.omega * moduleOffsets[i].x;

            double speed = Math.hypot(vx, vy);
            double angle = Math.atan2(vy, vx);
            states[i] = new SwerveModuleState(speed, angle);
        }
        return states;
    }
}
