package com.apexpathing.drivetrain;

import com.apexpathing.geometry.Vector;

import com.qualcomm.robotcore.hardware.DcMotorSimple;

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
    public double motorCachingThreshold = 0.01;
    public boolean useBrakeModeInTeleOp = false;
    public boolean useVoltageCompensation = false;
    public double nominalVoltage = 12.0;
    public double staticFrictionCoefficient = 0.1;

    public MecanumConstants() {
        defaults();
    }

    public MecanumConstants xVelocity(double xVelocity) {
        this.xVelocity = xVelocity;
        return this;
    }

    public MecanumConstants yVelocity(double yVelocity) {
        this.yVelocity = yVelocity;
        return this;
    }

    public MecanumConstants maxPower(double maxPower) {
        this.maxPower = maxPower;
        return this;
    }

    public MecanumConstants leftFrontMotorDirection(DcMotorSimple.Direction leftFrontMotorDirection) {
        this.leftFrontMotorDirection = leftFrontMotorDirection;
        return this;
    }

    public MecanumConstants leftRearMotorDirection(DcMotorSimple.Direction leftRearMotorDirection) {
        this.leftRearMotorDirection = leftRearMotorDirection;
        return this;
    }

    public MecanumConstants rightFrontMotorDirection(DcMotorSimple.Direction rightFrontMotorDirection) {
        this.rightFrontMotorDirection = rightFrontMotorDirection;
        return this;
    }

    public MecanumConstants rightRearMotorDirection(DcMotorSimple.Direction rightRearMotorDirection) {
        this.rightRearMotorDirection = rightRearMotorDirection;
        return this;
    }

    public MecanumConstants motorCachingThreshold(double motorCachingThreshold) {
        this.motorCachingThreshold = motorCachingThreshold;
        return this;
    }

    public MecanumConstants useBrakeModeInTeleOp(boolean useBrakeModeInTeleOp) {
        this.useBrakeModeInTeleOp = useBrakeModeInTeleOp;
        return this;
    }

    public MecanumConstants useVoltageCompensation(boolean useVoltageCompensation) {
        this.useVoltageCompensation = useVoltageCompensation;
        return this;
    }

    public MecanumConstants nominalVoltage(double nominalVoltage) {
        this.nominalVoltage = nominalVoltage;
        return this;
    }

    public MecanumConstants staticFrictionCoefficient(double staticFrictionCoefficient) {
        this.staticFrictionCoefficient = staticFrictionCoefficient;
        return this;
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
    public boolean isUseBrakeModeInTeleOp() { return useBrakeModeInTeleOp; }
    public void setUseBrakeModeInTeleOp(boolean v) { this.useBrakeModeInTeleOp = v; }

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
        useBrakeModeInTeleOp = false;
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
