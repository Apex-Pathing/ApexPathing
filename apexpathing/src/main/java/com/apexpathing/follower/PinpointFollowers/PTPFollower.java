package com.apexpathing.follower.PinpointFollowers;

import com.apexpathing.drivetrain.MecanumDrive;
import com.apexpathing.localization.PinpointLocalizer;
import com.apexpathing.util.math.Pose;
import com.apexpathing.util.math.Vector;

public class PTPFollower {

    private final MecanumDrive drive;
    private final PinpointLocalizer localizer;

    private Pose currentPose;
    private Pose targetPose;

    private double translationalKp = 0.03;
    private double headingKp = 0.5;

    private double translationalTolerance = 1.0;
    private double headingTolerance = 0.05;

    private double maxPower = 1.0;
    private double minPower = 0.05;

    private boolean isBusy = false;

    public PTPFollower(MecanumDrive drive, PinpointLocalizer localizer) {
        this.drive = drive;
        this.localizer = localizer;
    }

    public void setTarget(Pose target) {
        this.targetPose = target;
        this.isBusy = true;
    }

    public void update() {
        localizer.update();
        currentPose = localizer.getPose();

        if (currentPose == null || targetPose == null) {
            return;
        }

        double dx = targetPose.x() - currentPose.x();
        double dy = targetPose.y() - currentPose.y();

        double dist = Math.hypot(dx, dy);
        double headingError = normalizeAngle(targetPose.heading() - currentPose.heading());


        if (dist < translationalTolerance && Math.abs(headingError) < headingTolerance) {
            drive.botCentricDrive(0, 0, 0);
            isBusy = false;
            return;
        }

        Vector error = new Vector(dx, dy);
        error.rotateVec(-currentPose.heading());

        double x = error.getXComponent() * translationalKp;
        double y = error.getYComponent() * translationalKp;
        double turn = headingError * headingKp;

        double mag = Math.hypot(x, y);
        if (mag > maxPower) {
            x /= mag;
            y /= mag;
        }

        if (mag > 0) {
            x = applyMinPower(x);
            y = applyMinPower(y);
        }

        turn = clip(turn, -maxPower, maxPower);
        drive.botCentricDrive(x, y, turn);
    }

    public boolean isBusy() {
        return isBusy;
    }

    public Pose getPose() {
        return currentPose;
    }

    private double applyMinPower(double val) {
        if (Math.abs(val) < minPower) {
            return Math.signum(val) * minPower;
        }
        return val;
    }

    private double clip(double val, double min, double max) {
        return Math.max(min, Math.min(max, val));
    }

    private double normalizeAngle(double angle) {
        while (angle > Math.PI)  angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }
}