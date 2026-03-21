package com.xpathing.util.controllers;

import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * @author Xander Haemel
 */
public class PIDFController{
    public double kP,kI,kD,kF;
    public double goal = 0;
    double integralSum = 0;
    double derivitave;
    double lastError = 0;
    double error;
    double lastTimestamp = 0;
    ElapsedTime timer;
    public PIDFController(double kP, double kI, double kD, double kF){
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kF = kF;
        timer = new ElapsedTime();
        timer.startTime();
    }
    public void setGoal(double newGoal){
        this.goal = newGoal;
    }

    /**
     * the drive pidf for all our main controllers
     * @param currentPosition the current position of the robot
     * @return the power output in a double form
     */
    public synchronized double calculate(double currentPosition){
        //timers
        double timestamp = timer.milliseconds()/1000;
        double deltaTime = timestamp- lastTimestamp;

        //error calculation
        error = goal- currentPosition;

        //p constant
        double kPOut = kP * error;

        //i constant
        integralSum += error * deltaTime;

        // Cap the integral sum
        double iLimit = 0.25; // Maximum power the I-term can contribute (0.0 to 1.0)
        if (integralSum > iLimit) integralSum = iLimit;
        if (integralSum < -iLimit) integralSum = -iLimit;
        //i constant
        double kIOut = kI * integralSum;

        //d constant
        derivitave = (error - lastError)/deltaTime;
        double kDOut = kD * derivitave;

        //f constant
        double kFOut = kF * goal;

        //set last time
        lastTimestamp = timestamp;
        lastError = error;

        return kPOut + kIOut + kDOut + kFOut;

    }


}
