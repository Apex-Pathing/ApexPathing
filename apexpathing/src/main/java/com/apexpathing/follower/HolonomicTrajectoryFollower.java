package com.apexpathing.follower;

import com.apexpathing.util.math.Pose;
import com.apexpathing.util.math.Vector;

/**
 * A trajectory follower for holonomic robots (Mecanum, Swerve).
 * Uses PD feedback, acceleration feedforward, and centripetal force compensation.
  * @author Krish Joshi - 26192 Heatwaves
 */
public class HolonomicTrajectoryFollower {
    public static double kPx = 1.0;
    public static double kPy = 1.0;
    public static double kPtheta = 1.0;
    public static double kDx = 0.1;
    public static double kDy = 0.1;
    public static double kDtheta = 0.1;

    public static double kV = 0.02; // Velocity feedforward
    public static double kA = 0.01; // Acceleration feedforward
    public static double kCentripetal = 0.005; // Centripetal compensation

    private Pose lastError = new Pose(0, 0, 0);
    private long lastTime = -1;

    public HolonomicTrajectoryFollower() {
    }

    /**
     * Updates the follower and returns the target robot-relative velocities.
     * @param currentPose The robot's current pose from the localizer.
     * @param target The desired trajectory sample.
     * @return Target robot-relative velocities (vx, vy, vtheta) as a Pose.
     */
    public Pose update(Pose currentPose, TrajectorySample target) {
        long currentTime = System.nanoTime();
        if (lastTime == -1) lastTime = currentTime;
        double dt = (currentTime - lastTime) / 1e9;
        lastTime = currentTime;

        // Decouple translation and rotation for PD calculation
        double xError = target.pose.x() - currentPose.x();
        double yError = target.pose.y() - currentPose.y();
        double thetaError = normalizeAngle(target.pose.heading() - currentPose.heading());

        double dxError = (dt > 0) ? (xError - lastError.x()) / dt : 0;
        double dyError = (dt > 0) ? (yError - lastError.y()) / dt : 0;
        double dthetaError = (dt > 0) ? (thetaError - lastError.heading()) / dt : 0;

        lastError = new Pose(xError, yError, thetaError);

        // PD Feedback
        double feedbackX = kPx * xError + kDx * dxError;
        double feedbackY = kPy * yError + kDy * dyError;
        double feedbackTheta = kPtheta * thetaError + kDtheta * dthetaError;

        // Feedforward
        double feedforwardX = kV * target.velocity.x() + kA * target.acceleration.x();
        double feedforwardY = kV * target.velocity.y() + kA * target.acceleration.y();
        double feedforwardTheta = kV * target.velocity.heading() + kA * target.acceleration.heading();

        // Centripetal Force Compensation
        double centripetalX = -kCentripetal * target.velocity.y() * target.velocity.heading();
        double centripetalY = kCentripetal * target.velocity.x() * target.velocity.heading();

        // Total outputs in world-relative frame
        double worldVX = feedbackX + feedforwardX + centripetalX;
        double worldVY = feedbackY + feedforwardY + centripetalY;

        // Transform world velocity to robot-relative velocity
        double cos = Math.cos(currentPose.heading());
        double sin = Math.sin(currentPose.heading());

        double robotVX = worldVX * cos + worldVY * sin;
        double robotVY = -worldVX * sin + worldVY * cos;

        return new Pose(robotVX, robotVY, feedbackTheta + feedforwardTheta);
    }

    private double normalizeAngle(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }
}
