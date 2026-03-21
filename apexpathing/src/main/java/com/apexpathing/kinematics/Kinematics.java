package com.apexpathing.kinematics;

/**
 * Abstract Kinematics class.
 */
public abstract class Kinematics implements KinematicsSwitcher {
    @Override
    public abstract Object calculate(ChassisSpeeds chassisSpeeds);
}
