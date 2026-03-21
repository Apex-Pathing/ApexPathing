package com.apexpathing.hardware;

import com.qualcomm.robotcore.hardware.AnalogInput;

/**
 * Custom wrapper for analog absolute encoders to provide angle readings in radians.
 */
public class AbsoluteAnalogEncoder {
    private final AnalogInput encoder;
    private double offset = 0.0;
    private boolean inverted = false;
    private final double analogVoltage;

    public AbsoluteAnalogEncoder(AnalogInput encoder) {
        this(encoder, 3.3);
    }

    public AbsoluteAnalogEncoder(AnalogInput encoder, double analogVoltage) {
        this.encoder = encoder;
        this.analogVoltage = analogVoltage;
    }

    public AbsoluteAnalogEncoder zero(double offset) {
        this.offset = offset;
        return this;
    }

    public AbsoluteAnalogEncoder setInverted(boolean inverted) {
        this.inverted = inverted;
        return this;
    }

    public double getCurrentPosition() {
        double pos = (inverted ? -1.0 : 1.0) * ((encoder.getVoltage() / analogVoltage) * 2.0 * Math.PI);
        pos -= offset;
        return normalize(pos);
    }

    private double normalize(double angle) {
        angle = angle % (2.0 * Math.PI);
        if (angle < 0) angle += 2.0 * Math.PI;
        return angle;
    }

    public AnalogInput getEncoder() {
        return encoder;
    }
}
