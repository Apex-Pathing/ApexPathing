package com.apexpathing.kinematics;

/**
 * Abstract Kinematics class.
  * @author Krish Joshi - 26192 Heatwaves
 */
public abstract class Kinematics implements KinematicsSwitcher {
    @Override
    public abstract Object calculate(ChassisSpeeds chassisSpeeds);
}
