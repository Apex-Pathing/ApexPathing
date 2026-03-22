package com.apexpathing.drivetrain;

import com.apexpathing.util.math.Pose;
import com.apexpathing.localization.Localizer;
import com.apexpathing.follower.HolonomicTrajectoryFollower;
import com.apexpathing.follower.Trajectory;
import com.apexpathing.follower.TrajectorySample;
import com.qualcomm.robotcore.hardware.VoltageSensor;

/**
 * CustomDrive base with centralized update loop, voltage compensation,
 * and state machine hooks.
 * @author Krish Joshi - 26192 Heatwaves
 */
public abstract class CustomDrive {
    protected Localizer localizer;
    protected HolonomicTrajectoryFollower controller;
    protected VoltageSensor voltageSensor;

    protected double nominalVoltage = 12.0;
    protected boolean useVoltageCompensation = false;
    protected double staticFrictionCoefficient = 0.0;

    protected boolean isFollowing = false;
    protected Trajectory currentTrajectory;
    protected long startTime;

    /**
     * Translates a robot-relative velocity vector into specific motor commands.
     * @param drivePowers The target Pose representing (vx, vy, vtheta).
     */
    public abstract void setDrivePowers(Pose drivePowers);

    /**
     * Translates manual inputs (x, y, rx) into a Pose and sets drive powers.
     * @param args Array of doubles such that [0] == x, [1] == y, [2] == rx.
     */
    public void drive(double ...args) {
        if (args.length >= 3) {
            setDrivePowers(new Pose(args[0], args[1], args[2]));
        }
    }

    /**
     * Rotates the robot in place.
     * @param power The turning power.
     */
    public void turn(double power) {
        drive(0, 0, power);
    }

    /**
     * Update loop called once per OpMode loop.
     * Centralizes localizer updates and trajectory following logic.
     */
    public void update() {
        if (localizer != null) {
            localizer.update();
        }

        if (isFollowing && currentTrajectory != null) {
            double elapsedTime = (System.nanoTime() - startTime) / 1e9;
            if (elapsedTime <= currentTrajectory.duration()) {
                TrajectorySample target = currentTrajectory.sample(elapsedTime);
                Pose currentPose = localizer != null ? localizer.getPose() : new Pose(0, 0, 0);
                Pose powers = controller.update(currentPose, target);

                if (useVoltageCompensation) {
                    double vComp = getVoltageCompensationMultiplier();
                    powers = new Pose(powers.x() * vComp, powers.y() * vComp, powers.heading() * vComp);
                }

                setDrivePowers(powers);
            } else {
                isFollowing = false;
                currentTrajectory = null;
                setDrivePowers(new Pose(0, 0, 0));
            }
        }
    }

    /**
     * Start following a trajectory.
     * @param traj The trajectory to follow.
     */
    public void followTrajectory(Trajectory traj) {
        this.currentTrajectory = traj;
        this.startTime = System.nanoTime();
        this.isFollowing = true;
    }

    /**
     * @return Whether the robot is currently following a trajectory.
     */
    public boolean isBusy() {
        return isFollowing;
    }

    /**
     * @return The robot's current pose from the localizer.
     */
    public Pose getPose() {
        return localizer != null ? localizer.getPose() : new Pose(0, 0, 0);
    }

    /**
     * Sets the robot's current pose.
     * @param pose The new pose.
     */
    public void setPose(Pose pose) {
        if (localizer != null) {
            localizer.setPose(pose);
        }
    }

    /**
     * Calculates the voltage compensation multiplier.
     * @return The multiplier to apply to motor powers.
     */
    protected double getVoltageCompensationMultiplier() {
        if (voltageSensor == null) return 1.0;
        double voltage = voltageSensor.getVoltage();
        if (voltage < 1.0) return 1.0;

        return (nominalVoltage - (nominalVoltage * staticFrictionCoefficient)) /
                (voltage - ((nominalVoltage * nominalVoltage / voltage) * staticFrictionCoefficient));
    }

    public void setNominalVoltage(double nominalVoltage) {
        this.nominalVoltage = nominalVoltage;
    }

    public void setUseVoltageCompensation(boolean useVoltageCompensation) {
        this.useVoltageCompensation = useVoltageCompensation;
    }

    public void setStaticFrictionCoefficient(double staticFrictionCoefficient) {
        this.staticFrictionCoefficient = staticFrictionCoefficient;
    }
}
