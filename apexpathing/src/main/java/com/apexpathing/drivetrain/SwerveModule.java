package com.apexpathing.drivetrain;

import com.qualcomm.robotcore.hardware.Servo;
import com.apexpathing.hardware.MotorEx;
import com.apexpathing.hardware.AbsoluteAnalogEncoder;

/**
 * Hardware wrapper handling a drive motor, turn motor (servo), and absolute encoder.
 */
public class SwerveModule {
    private final MotorEx driveMotor;
    private final Servo turnServo;
    private final AbsoluteAnalogEncoder absoluteEncoder;

    private double lastPower = 0.0;
    private double lastPosition = -1.0;
    private static final double CACHE_THRESHOLD = 0.01;
    private static final double MIN_VELOCITY_FOR_ROTATION = 0.02;

    public SwerveModule(MotorEx driveMotor, Servo turnServo, AbsoluteAnalogEncoder absoluteEncoder) {
        this.driveMotor = driveMotor;
        this.turnServo = turnServo;
        this.absoluteEncoder = absoluteEncoder;
    }

    /**
     * Optimized method to set the module's target velocity and angle.
     * Includes angle normalization, cosine optimization (wheel flip), and hardware caching.
     *
     * @param targetVelocity Target velocity from -1.0 to 1.0.
     * @param targetAngleRadians Target angle in radians.
     */
    public void setDesiredState(double targetVelocity, double targetAngleRadians) {
        setDesiredState(targetVelocity, targetAngleRadians, false);
    }

    /**
     * Optimized method to set the module's target velocity and angle.
     * Includes angle normalization, cosine optimization (wheel flip), and hardware caching.
     *
     * @param targetVelocity Target velocity from -1.0 to 1.0.
     * @param targetAngleRadians Target angle in radians.
     * @param forceAngle If true, updates the servo position even if velocity is zero.
     */
    public void setDesiredState(double targetVelocity, double targetAngleRadians, boolean forceAngle) {
        targetAngleRadians = normalizeAngle(targetAngleRadians);

        double currentAngle = getModuleAngle();
        double delta = normalizeAngleDifference(targetAngleRadians - currentAngle);

        if (Math.abs(delta) > Math.PI / 2.0) {
            targetVelocity *= -1.0;
            targetAngleRadians = normalizeAngle(targetAngleRadians + Math.PI);
        }

        double targetPosition = targetAngleRadians / (2.0 * Math.PI);

        if (Math.abs(targetVelocity - lastPower) > CACHE_THRESHOLD) {
            driveMotor.setPower(targetVelocity);
            lastPower = targetVelocity;
        }

        // Only update angle if moving or forced, to avoid wheels snapping to 0.
        if (forceAngle || Math.abs(targetVelocity) > MIN_VELOCITY_FOR_ROTATION) {
            if (Math.abs(targetPosition - lastPosition) > CACHE_THRESHOLD) {
                turnServo.setPosition(targetPosition);
                lastPosition = targetPosition;
            }
        }
    }

    /**
     * Sets the drive motor power directly.
     * @param power The motor power.
     */
    public void setMotorPower(double power) {
        if (Math.abs(power - lastPower) > CACHE_THRESHOLD) {
            driveMotor.setPower(power);
            lastPower = power;
        }
    }

    /**
     * Returns the current module angle in radians from the absolute encoder.
     */
    public double getModuleAngle() {
        return absoluteEncoder.getCurrentPosition();
    }

    /**
     * Returns the current wheel position (e.g., in encoder ticks or distance).
     */
    public double getWheelPosition() {
        return driveMotor.getCurrentPosition();
    }

    private double normalizeAngle(double angle) {
        angle = angle % (2.0 * Math.PI);
        if (angle < 0) angle += 2.0 * Math.PI;
        return angle;
    }

    private double normalizeAngleDifference(double delta) {
        while (delta > Math.PI) delta -= 2.0 * Math.PI;
        while (delta < -Math.PI) delta += 2.0 * Math.PI;
        return delta;
    }

    public double getLastPower() { return lastPower; }
    public double getLastPosition() { return lastPosition; }

    //for swerve
    public AbsoluteAnalogEncoder getEncoder() { return absoluteEncoder; }
}
