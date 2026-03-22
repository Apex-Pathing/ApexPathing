package com.apexpathing.kinematics;

/**
 * Specialized TankKinematics class.
  * @author Krish Joshi - 26192 Heatwaves
 */
public class TankKinematics extends Kinematics {
    private final double trackWidth;

    public TankKinematics(double trackWidth) {
        this.trackWidth = trackWidth;
    }

    public double[] calculateWheelSpeeds(ChassisSpeeds chassisSpeeds) {
        double leftSpeed = chassisSpeeds.vx - (trackWidth / 2.0) * chassisSpeeds.omega;
        double rightSpeed = chassisSpeeds.vx + (trackWidth / 2.0) * chassisSpeeds.omega;

        return new double[]{leftSpeed, rightSpeed};
    }

    @Override
    public double[] calculate(ChassisSpeeds chassisSpeeds) {
        return calculateWheelSpeeds(chassisSpeeds);
    }
}
