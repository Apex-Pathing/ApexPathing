package com.xpathing.drivetrains;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.movement.hardware.MotorEx;

/**
 * Hardware wrapper handling a drive motor, turn motor (servo), and absolute encoder.
 */
public class SwerveModule {
    private final MotorEx driveMotor;
    private final Servo turnServo;
    private final AnalogInput absoluteEncoder;

    private double lastPower = 0.0;
    private double lastPosition = -1.0;
    private static final double CACHE_THRESHOLD = 0.01;

    public SwerveModule(MotorEx driveMotor, Servo turnServo, AnalogInput absoluteEncoder) {
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
        // 1. Angle Normalization: Ensure the incoming targetAngleRadians is between 0 and 2pi
        targetAngleRadians = normalizeAngle(targetAngleRadians);

        // 2. Cosine Optimization (Wheel Flip)
        double currentAngle = getModuleAngle();
        double delta = normalizeAngleDifference(targetAngleRadians - currentAngle);

        // If the angular distance is greater than pi/2 (90 degrees), flip velocity and adjust angle by pi
        if (Math.abs(delta) > Math.PI / 2.0) {
            targetVelocity *= -1.0;
            targetAngleRadians = normalizeAngle(targetAngleRadians + Math.PI);
        }

        // 3. Servo Mapping: Convert final targetAngleRadians into 0.0 to 1.0 range (0 = 0.0, 2pi = 1.0)
        double targetPosition = targetAngleRadians / (2.0 * Math.PI);

        // 4. Caching: Only call hardware if values changed significantly
        if (Math.abs(targetVelocity - lastPower) > CACHE_THRESHOLD) {
            driveMotor.setPower(targetVelocity);
            lastPower = targetVelocity;
        }

        if (Math.abs(targetPosition - lastPosition) > CACHE_THRESHOLD) {
            turnServo.setPosition(targetPosition);
            lastPosition = targetPosition;
        }
    }

    /**
     * Returns the current module angle in radians from the absolute encoder.
     */
    public double getModuleAngle() {
        // Assuming the absolute encoder returns 0-3.3V mapped to 0-2pi
        return (absoluteEncoder.getVoltage() / 3.3) * 2.0 * Math.PI;
    }

    /**
     * Returns the current wheel position (e.g., in encoder ticks or distance).
     */
    public double getWheelPosition() {
        return driveMotor.getCurrentPosition();
    }

    /**
     * Helper to normalize angle to [0, 2pi)
     */
    private double normalizeAngle(double angle) {
        angle = angle % (2.0 * Math.PI);
        if (angle < 0) angle += 2.0 * Math.PI;
        return angle;
    }

    /**
     * Helper to normalize angle difference to [-pi, pi]
     */
    private double normalizeAngleDifference(double delta) {
        while (delta > Math.PI) delta -= 2.0 * Math.PI;
        while (delta < -Math.PI) delta += 2.0 * Math.PI;
        return delta;
    }

    public void update() {
        // Local module update logic (e.g., PID if using a motor for turn, but here we use a servo)
    }

    public double getLastPower() { return lastPower; }
    public double getLastPosition() { return lastPosition; }
}
