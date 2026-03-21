package com.apexpathing.follower;

import com.apexpathing.geometry.Vector2d;

import java.util.ArrayList;
import java.util.List;

/**
 * A utility to parameterize a spline by arc length.
 * Maps arc length distance s to spline parameter t.
 */
public class ArcLengthParameterizer {
    private static final int SAMPLES = 100;

    private final QuinticHermiteSpline spline;
    private final List<Double> arcLengths;
    private final double totalArcLength;

    public ArcLengthParameterizer(QuinticHermiteSpline spline) {
        this.spline = spline;
        this.arcLengths = new ArrayList<>(SAMPLES + 1);
        this.arcLengths.add(0.0);

        double accumulatedLength = 0.0;
        Vector2d prevPoint = spline.getPoint(0);

        for (int i = 1; i <= SAMPLES; i++) {
            double t = (double) i / SAMPLES;
            Vector2d currentPoint = spline.getPoint(t);
            accumulatedLength += prevPoint.distanceTo(currentPoint);
            arcLengths.add(accumulatedLength);
            prevPoint = currentPoint;
        }

        this.totalArcLength = accumulatedLength;
    }

    /**
     * Maps arc length distance s to spline parameter t.
     * @param s Distance along the path (0 to totalArcLength).
     * @return Spline parameter t (0 to 1).
     */
    public double getT(double s) {
        if (s <= 0) return 0;
        if (s >= totalArcLength) return 1;

        int low = 0;
        int high = SAMPLES;
        while (low < high - 1) {
            int mid = (low + high) / 2;
            if (arcLengths.get(mid) < s) {
                low = mid;
            } else {
                high = mid;
            }
        }

        double s0 = arcLengths.get(low);
        double s1 = arcLengths.get(high);
        double t0 = (double) low / SAMPLES;
        double t1 = (double) high / SAMPLES;

        return t0 + (s - s0) / (s1 - s0) * (t1 - t0);
    }

    public double getTotalArcLength() {
        return totalArcLength;
    }
}
