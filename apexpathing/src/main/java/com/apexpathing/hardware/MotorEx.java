package com.apexpathing.hardware;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.MotorControlAlgorithm;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import com.apexpathing.util.SlewRateLimiter;

/**
 * Enhanced DcMotorEx wrapper with power caching and slew rate limiting.
 */
public class MotorEx implements DcMotorEx {
    public static double POWER_CACHE_THRESHOLD = 0.005;

    private final DcMotorEx motor;
    private double lastPower = 0.0;
    private SlewRateLimiter slewRateLimiter;
    private boolean slewRateEnabled = false;

    public MotorEx(DcMotorEx motor) {
        this.motor = motor;
    }

    public void setSlewRateLimit(double limit) {
        this.slewRateLimiter = new SlewRateLimiter(limit);
        this.slewRateEnabled = true;
    }

    public void disableSlewRate() {
        this.slewRateEnabled = false;
    }

    @Override
    public void setPower(double power) {
        double targetPower = power;
        if (slewRateEnabled && slewRateLimiter != null) {
            targetPower = slewRateLimiter.calculate(power);
        }

        if (Math.abs(targetPower - lastPower) > POWER_CACHE_THRESHOLD) {
            motor.setPower(targetPower);
            lastPower = targetPower;
        }
    }

    @Override
    public double getPower() {
        return lastPower;
    }

    // --- DcMotorEx Delegates ---

    @Override public void setMotorEnable() { motor.setMotorEnable(); }
    @Override public void setMotorDisable() { motor.setMotorDisable(); }
    @Override public boolean isMotorEnabled() { return motor.isMotorEnabled(); }
    @Override public void setVelocity(double angularRate) { motor.setVelocity(angularRate); }
    @Override public void setVelocity(double angularRate, AngleUnit unit) { motor.setVelocity(angularRate, unit); }
    @Override public double getVelocity() { return motor.getVelocity(); }
    @Override public double getVelocity(AngleUnit unit) { return motor.getVelocity(unit); }
    @Override public void setPIDCoefficients(RunMode mode, PIDCoefficients pidCoefficients) { motor.setPIDCoefficients(mode, pidCoefficients); }
    @Override public void setPIDFCoefficients(RunMode mode, PIDFCoefficients pidfCoefficients) { motor.setPIDFCoefficients(mode, pidfCoefficients); }
    @Override public void setVelocityPIDFCoefficients(double p, double i, double d, double f) { motor.setVelocityPIDFCoefficients(p, i, d, f); }
    @Override public void setPositionPIDFCoefficients(double p) { motor.setPositionPIDFCoefficients(p); }
    @Override public PIDCoefficients getPIDCoefficients(RunMode mode) { return motor.getPIDCoefficients(mode); }
    @Override public PIDFCoefficients getPIDFCoefficients(RunMode mode) { return motor.getPIDFCoefficients(mode); }
    @Override public void setTargetPositionTolerance(int tolerance) { motor.setTargetPositionTolerance(tolerance); }
    @Override public int getTargetPositionTolerance() { return motor.getTargetPositionTolerance(); }
    @Override public double getCurrent(CurrentUnit unit) { return motor.getCurrent(unit); }
    @Override public double getCurrentAlert(CurrentUnit unit) { return motor.getCurrentAlert(unit); }
    @Override public void setCurrentAlert(double current, CurrentUnit unit) { motor.setCurrentAlert(current, unit); }
    @Override public boolean isOverCurrent() { return motor.isOverCurrent(); }

    // --- DcMotor Delegates ---

    @Override public MotorConfigurationType getMotorType() { return motor.getMotorType(); }
    @Override public void setMotorType(MotorConfigurationType type) { motor.setMotorType(type); }
    @Override public com.qualcomm.robotcore.hardware.DcMotorController getController() { return motor.getController(); }
    @Override public int getPortNumber() { return motor.getPortNumber(); }
    @Override public void setZeroPowerBehavior(ZeroPowerBehavior zeroPowerBehavior) { motor.setZeroPowerBehavior(zeroPowerBehavior); }
    @Override public ZeroPowerBehavior getZeroPowerBehavior() { return motor.getZeroPowerBehavior(); }
    @Override public void setPowerFloat() { motor.setPowerFloat(); }
    @Override public boolean getPowerFloat() { return motor.getPowerFloat(); }
    @Override public void setTargetPosition(int position) { motor.setTargetPosition(position); }
    @Override public int getTargetPosition() { return motor.getTargetPosition(); }
    @Override public boolean isBusy() { return motor.isBusy(); }
    @Override public int getCurrentPosition() { return motor.getCurrentPosition(); }
    @Override public void setMode(RunMode mode) { motor.setMode(mode); }
    @Override public RunMode getMode() { return motor.getMode(); }
    @Override public void setDirection(Direction direction) { motor.setDirection(direction); }
    @Override public Direction getDirection() { return motor.getDirection(); }

    // --- HardwareDevice Delegates ---

    @Override public Manufacturer getManufacturer() { return motor.getManufacturer(); }
    @Override public String getDeviceName() { return motor.getDeviceName(); }
    @Override public String getConnectionInfo() { return motor.getConnectionInfo(); }
    @Override public int getVersion() { return motor.getVersion(); }
    @Override public void resetDeviceConfigurationForOpMode() { motor.resetDeviceConfigurationForOpMode(); }
    @Override public void close() { motor.close(); }
}
