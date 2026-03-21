package com.apexpathing.kinematics;

/**
 * Specialized MecanumKinematics class.
 */
public class MecanumKinematics extends Kinematics {
    private final double trackWidth;
    private final double wheelBase;
    private final double lateralMultiplier;

    /**
     * @param trackWidth Distance between left and right wheels.
     * @param wheelBase Distance between front and back wheels.
     */
    public MecanumKinematics(double trackWidth, double wheelBase, double lateralMultiplier) {
        this.trackWidth = trackWidth;
        this.wheelBase = wheelBase;
        this.lateralMultiplier = lateralMultiplier;
    }

    public double[] calculateWheelSpeeds(ChassisSpeeds chassisSpeeds) {
        double vx = chassisSpeeds.vx;
        double vy = chassisSpeeds.vy * lateralMultiplier;
        double omega = chassisSpeeds.omega;

        double k = (trackWidth + wheelBase) / 2.0;

        double fl = vx - vy - k * omega;
        double fr = vx + vy + k * omega;
        double bl = vx + vy - k * omega;
        double br = vx - vy + k * omega;

        return new double[]{fl, fr, bl, br};
    }

    @Override
    public double[] calculate(ChassisSpeeds chassisSpeeds) {
        return calculateWheelSpeeds(chassisSpeeds);
    }
}
