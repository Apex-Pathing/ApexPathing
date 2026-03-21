package com.apexpathing.util;

import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * PIDF feedback controller.
 */
public class PIDFController {
    public double kP, kI, kD, kF;
    public double goal = 0;
    private double integralSum = 0;
    private double derivative;
    private double lastError = 0;
    private double error;
    private double lastTimestamp = 0;
    private final ElapsedTime timer;

    public PIDFController(double kP, double kI, double kD, double kF) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kF = kF;
        timer = new ElapsedTime();
        timer.startTime();
    }

    public void setGoal(double newGoal) {
        this.goal = newGoal;
    }

    /**
     * Calculates the PIDF output.
     * @param currentPosition the current position of the mechanism
     * @return the power output
     */
    public synchronized double calculate(double currentPosition) {
        double timestamp = timer.milliseconds() / 1000;
        double deltaTime = timestamp - lastTimestamp;

        if (deltaTime == 0) return 0;

        error = goal - currentPosition;

        double kPOut = kP * error;

        integralSum += error * deltaTime;
        double iLimit = 0.25;
        if (integralSum > iLimit) integralSum = iLimit;
        if (integralSum < -iLimit) integralSum = -iLimit;
        double kIOut = kI * integralSum;

        derivative = (error - lastError) / deltaTime;
        double kDOut = kD * derivative;

        double kFOut = kF * goal;

        lastTimestamp = timestamp;
        lastError = error;

        return kPOut + kIOut + kDOut + kFOut;
    }
}
