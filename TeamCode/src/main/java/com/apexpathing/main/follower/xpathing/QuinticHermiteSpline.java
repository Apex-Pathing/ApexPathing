package com.apexpathing.main.follower.xpathing;

import org.firstinspires.ftc.teamcode.movement.geometry.Vector2d;

/**
 * A quintic Hermite spline solver and evaluator.
 * Solves for 5th-degree polynomial coefficients given start/end poses, velocities, and accelerations.
 */
public class QuinticHermiteSpline {
    private final double[] xCoeffs;
    private final double[] yCoeffs;

    /**
     * Solves for 5th-degree polynomial coefficients.
     * The spline is parameterized from t=0 to t=1.
     */
    public QuinticHermiteSpline(
            Vector2d startPos, Vector2d startVel, Vector2d startAccel,
            Vector2d endPos, Vector2d endVel, Vector2d endAccel
    ) {
        this.xCoeffs = solve(startPos.x, startVel.x, startAccel.x, endPos.x, endVel.x, endAccel.x);
        this.yCoeffs = solve(startPos.y, startVel.y, startAccel.y, endPos.y, endVel.y, endAccel.y);
    }

    /**
     * Solves for coefficients a, b, c, d, e, f of f(t) = at^5 + bt^4 + ct^3 + dt^2 + et + f
     * given f(0), f'(0), f''(0), f(1), f'(1), f''(1).
     */
    private double[] solve(double p0, double v0, double a0, double p1, double v1, double a1) {
        // f(0) = f = p0
        // f'(0) = e = v0
        // f''(0) = 2d = a0 => d = a0 / 2
        double f = p0;
        double e = v0;
        double d = a0 / 2.0;

        // At t=1:
        // a + b + c + d + e + f = p1
        // 5a + 4b + 3c + 2d + e = v1
        // 20a + 12b + 6c + 2d = a1

        // Substitute d, e, f:
        // a + b + c = p1 - p0 - v0 - a0/2
        // 5a + 4b + 3c = v1 - v0 - a0
        // 20a + 12b + 6c = a1 - a0

        double targetP = p1 - p0 - v0 - a0 / 2.0;
        double targetV = v1 - v0 - a0;
        double targetA = a1 - a0;

        // Solve:
        // [ 1  1  1 ] [ a ]   [ targetP ]
        // [ 5  4  3 ] [ b ] = [ targetV ]
        // [ 20 12 6 ] [ c ]   [ targetA ]

        // Using inverse matrix:
        // |  1  1  1 |^-1     |  6 -3  0.5 |
        // |  5  4  3 |    =   | -15 7 -1 |
        // | 20 12  6 |        |  10 -4 0.5 | * (-1/??) Wait, let's just do manual elimination or direct formula.

        // Determinant = 1(24-36) - 1(30-60) + 1(60-80) = -12 + 30 - 20 = -2
        // a = (targetP * (24-36) - (targetV * (6-12)) + (targetA * (3-4))) / -2
        // a = (targetP * -12 - targetV * -6 + targetA * -1) / -2
        // a = 6*targetP - 3*targetV + 0.5*targetA

        // b = (-targetP * (30-60) + targetV * (6-20) - targetA * (3-5)) / -2
        // b = (-targetP * -30 + targetV * -14 - targetA * -2) / -2
        // b = -15*targetP + 7*targetV - 1*targetA

        // c = (targetP * (60-80) - targetV * (12-20) + targetA * (4-5)) / -2
        // c = (targetP * -20 - targetV * -8 + targetA * -1) / -2
        // c = 10*targetP - 4*targetV + 0.5*targetA

        double a = 6 * targetP - 3 * targetV + 0.5 * targetA;
        double b = -15 * targetP + 7 * targetV - targetA;
        double c = 10 * targetP - 4 * targetV + 0.5 * targetA;

        return new double[]{a, b, c, d, e, f};
    }

    public Vector2d getPoint(double t) {
        return new Vector2d(evaluate(xCoeffs, t), evaluate(yCoeffs, t));
    }

    public Vector2d getVelocity(double t) {
        return new Vector2d(evaluateDerivative(xCoeffs, t), evaluateDerivative(yCoeffs, t));
    }

    public Vector2d getAcceleration(double t) {
        return new Vector2d(evaluateSecondDerivative(xCoeffs, t), evaluateSecondDerivative(yCoeffs, t));
    }

    private double evaluate(double[] coeffs, double t) {
        return ((((coeffs[0] * t + coeffs[1]) * t + coeffs[2]) * t + coeffs[3]) * t + coeffs[4]) * t + coeffs[5];
    }

    private double evaluateDerivative(double[] coeffs, double t) {
        return (((5 * coeffs[0] * t + 4 * coeffs[1]) * t + 3 * coeffs[2]) * t + 2 * coeffs[3]) * t + coeffs[4];
    }

    private double evaluateSecondDerivative(double[] coeffs, double t) {
        return ((20 * coeffs[0] * t + 12 * coeffs[1]) * t + 6 * coeffs[2]) * t + 2 * coeffs[3];
    }
}
