package com.apexpathing.util;

/**
 * A class that limits the rate of change of a value.
 */
public class SlewRateLimiter {
    private double rateLimit;
    private double lastValue;
    private long lastTime;

    /**
     * @param rateLimit Units per second.
     */
    public SlewRateLimiter(double rateLimit) {
        this.rateLimit = rateLimit;
        this.lastTime = System.nanoTime();
    }

    public void setRateLimit(double rateLimit) {
        this.rateLimit = rateLimit;
    }

    public void reset(double value) {
        this.lastValue = value;
        this.lastTime = System.nanoTime();
    }

    public double calculate(double targetValue) {
        long currentTime = System.nanoTime();
        double dt = (currentTime - lastTime) / 1e9;
        lastTime = currentTime;

        double maxChange = rateLimit * dt;
        double change = targetValue - lastValue;

        if (change > maxChange) {
            lastValue += maxChange;
        } else if (change < -maxChange) {
            lastValue -= maxChange;
        } else {
            lastValue = targetValue;
        }

        return lastValue;
    }

    public double getLastValue() {
        return lastValue;
    }
}
