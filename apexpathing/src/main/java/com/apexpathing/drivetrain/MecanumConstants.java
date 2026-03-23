package com.apexpathing.drivetrain;

import com.apexpathing.util.math.Vector;

import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * @author Krish Joshi - 26192 Heatwaves
 */
public class MecanumConstants {
    public double xVelocity = 81.34056;
    public double yVelocity = 65.43028;

    private double[] convertToPolar = cartesianToPolar(xVelocity, -yVelocity);

    public Vector frontLeftVector = new Vector(convertToPolar[0], convertToPolar[1]).normalize();
    public double maxPower = 1;
    public DcMotorSimple.Direction leftFrontMotorDirection = DcMotorSimple.Direction.REVERSE;
    public DcMotorSimple.Direction leftRearMotorDirection = DcMotorSimple.Direction.REVERSE;
    public DcMotorSimple.Direction rightFrontMotorDirection = DcMotorSimple.Direction.FORWARD;
    public DcMotorSimple.Direction rightRearMotorDirection = DcMotorSimple.Direction.FORWARD;
    public String leftFrontMotorName = "leftFront";
    public String rightFrontMotorName = "rightFront";
    public String leftRearMotorName = "leftRear";
    public String rightRearMotorName = "rightRear";
    public double motorCachingThreshold = 0.01;
    public boolean useBrakeMode = false;
    public boolean useVoltageCompensation = false;
    public double nominalVoltage = 12.0;
    public double staticFrictionCoefficient = 0.1;

    public MecanumConstants() {
        defaults();
    }
    public double getXVelocity() { return xVelocity; }
    public void setXVelocity(double xVelocity) { this.xVelocity = xVelocity; }
    public double getYVelocity() { return yVelocity; }
    public void setYVelocity(double yVelocity) { this.yVelocity = yVelocity; }
    public Vector getFrontLeftVector() { return frontLeftVector; }
    public void setFrontLeftVector(Vector frontLeftVector) { this.frontLeftVector = frontLeftVector; }
    public double getMaxPower() { return maxPower; }
    public void setMaxPower(double maxPower) { this.maxPower = maxPower; }
    public DcMotorSimple.Direction getLeftFrontMotorDirection() { return leftFrontMotorDirection; }
    public void setLeftFrontMotorDirection(DcMotorSimple.Direction d) { this.leftFrontMotorDirection = d; }
    public DcMotorSimple.Direction getLeftRearMotorDirection() { return leftRearMotorDirection; }
    public void setLeftRearMotorDirection(DcMotorSimple.Direction d) { this.leftRearMotorDirection = d; }
    public DcMotorSimple.Direction getRightFrontMotorDirection() { return rightFrontMotorDirection; }
    public void setRightFrontMotorDirection(DcMotorSimple.Direction d) { this.rightFrontMotorDirection = d; }
    public DcMotorSimple.Direction getRightRearMotorDirection() { return rightRearMotorDirection; }
    public void setRightRearMotorDirection(DcMotorSimple.Direction d) { this.rightRearMotorDirection = d; }
    public double getMotorCachingThreshold() { return motorCachingThreshold; }
    public void setMotorCachingThreshold(double v) { this.motorCachingThreshold = v; }

    public void defaults() {
        xVelocity = 81.34056;
        yVelocity = 65.43028;
        convertToPolar = cartesianToPolar(xVelocity, -yVelocity);
        frontLeftVector = new Vector(convertToPolar[0], convertToPolar[1]).normalize();
        maxPower = 1;
        leftFrontMotorDirection = DcMotorSimple.Direction.REVERSE;
        leftRearMotorDirection = DcMotorSimple.Direction.REVERSE;
        rightFrontMotorDirection = DcMotorSimple.Direction.FORWARD;
        rightRearMotorDirection = DcMotorSimple.Direction.FORWARD;
        motorCachingThreshold = 0.01;
        useBrakeMode = false;
        useVoltageCompensation = false;
        nominalVoltage = 12.0;
        staticFrictionCoefficient = 0.1;
    }

    private static double[] cartesianToPolar(double x, double y) {
        double r = Math.sqrt(x * x + y * y);
        double theta = Math.atan2(y, x);
        return new double[]{r, theta};
    }
}
