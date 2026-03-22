package com.apexpathing.follower;

import com.apexpathing.drivetrain.MecanumDrive;
import com.apexpathing.util.math.Vector;
import com.apexpathing.util.math.Pose;


/**
 * Basic point-to-point follower class
 * @author Krish Joshi - 26192 Heatwaves
 * @author Sohum Arora 22985 Paraducks
 */
public class P2PFollower {

    private final MecanumDrive drive;

    private Pose currentPose;
    private Pose targetPose;

    private double translationalKp = 0.1;
    private double headingKp = 0.5;
    private double translationalTolerance = 1.0;
    private double headingTolerance= 0.05;

    public P2PFollower(MecanumDrive drive) {
        this.drive = drive;
    }

    public void setTarget(Pose target) {
        this.targetPose = target;
    }

    public void setCurrentPose(Pose pose) {
        this.currentPose = pose;
    }

    public void setTranslationalKp(double kp) { this.translationalKp = kp; }
    public void setHeadingKp(double kp) { this.headingKp = kp; }
    public void setTranslationalTolerance(double t) { this.translationalTolerance = t; }
    public void setHeadingTolerance(double t) { this.headingTolerance = t; }

    public void update() {
        if (currentPose != null && targetPose != null) {

            double dx = targetPose.x() - currentPose.x();
            double dy = targetPose.y() - currentPose.y();

            Vector error = new Vector(dx, dy);

            double headingError = normalizeAngle(targetPose.heading() - currentPose.heading());

            Vector rotated = error.copy();
            rotated.rotateVec(-currentPose.heading());

            double x = rotated.getXComponent() * translationalKp;
            double y = rotated.getYComponent() * translationalKp;
            double turn = headingError * headingKp;

            drive.botCentricDrive(x, y, turn);
        }
    }

    public boolean isAtTarget() {
        if (currentPose == null || targetPose == null) return false;
        double dx = targetPose.x() - currentPose.x();
        double dy = targetPose.y() - currentPose.y();
        double dist = Math.hypot(dx, dy);
        double headingError = Math.abs(normalizeAngle(targetPose.heading() - currentPose.heading()));
        return dist < translationalTolerance && headingError < headingTolerance;
    }

    private double normalizeAngle(double angle) {
        while (angle > Math.PI)  angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }
}
