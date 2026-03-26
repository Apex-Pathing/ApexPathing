package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@Autonomous(name="Custom Quintic Spline Path", group="Linear Opmode")
public class QuinticCurve extends LinearOpMode {

    // --- HARDWARE ---
    private DcMotorEx frontLeft, frontRight, backLeft, backRight;
    private GoBildaPinpointDriver pinpoint;

    // --- TUNING PARAMETERS ---
    // Proportional gain for fixing position drift.
    // Increase if robot trails the path, decrease if it jitters.
    private double Kp_xy = 0.08;
    private double Kp_heading = 0.8;

    @Override
    public void runOpMode() {
        // 1. Initialize Hardware
        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
        backRight = hardwareMap.get(DcMotorEx.class, "backRight");

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        // Initialize goBILDA Pinpoint
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.setOffsets(0.0,0.0, DistanceUnit.INCH); // UPDATE THIS to your specific pod offsets in mm
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        pinpoint.resetPosAndIMU();

        // 2. Define the Waypoints (Adjustable States)
        // Format: State(Position, Velocity, Acceleration)

        // --- STARTING POINT ---
        State startX = new State(0, 0, 0);
        State startY = new State(0, 0, 0);

        // --- WAYPOINT (Middle) ---
        // Passing through (24, 24) with a velocity aimed towards the top right
        State waypointX = new State(24, 40, 0);
        State waypointY = new State(24, 20, 0);

        // --- END POINT ---
        // Stopping at (48, 48)
        State endX = new State(48, 0, 0);
        State endY = new State(48, 0, 0);

        // Time allocated for each segment (in seconds)
        double timeSeg1 = 2.0; // Start -> Waypoint
        double timeSeg2 = 2.0; // Waypoint -> End

        // Create the Splines
        QuinticPolynomial spline1_X = new QuinticPolynomial(startX, waypointX, timeSeg1);
        QuinticPolynomial spline1_Y = new QuinticPolynomial(startY, waypointY, timeSeg1);

        QuinticPolynomial spline2_X = new QuinticPolynomial(waypointX, endX, timeSeg2);
        QuinticPolynomial spline2_Y = new QuinticPolynomial(waypointY, endY, timeSeg2);

        telemetry.addData("Status", "Initialized. Math Calculated.");
        telemetry.update();

        waitForStart();

        // 3. Execute Segment 1
        followSpline(spline1_X, spline1_Y, timeSeg1);

        // 4. Execute Segment 2
        followSpline(spline2_X, spline2_Y, timeSeg2);

        // Stop Motors
        setMotorPowers(0, 0, 0, 0);
    }

    /**
     * Feedforward + Proportional Controller to follow a specific spline over time T.
     */
    private void followSpline(QuinticPolynomial xSpline, QuinticPolynomial ySpline, double T) {
        ElapsedTime timer = new ElapsedTime();
        timer.reset();

        while (opModeIsActive() && timer.seconds() < T) {
            double t = timer.seconds();

            // Update Odometry
            pinpoint.update();
            Pose2D pose = pinpoint.getPosition();
            double currentX = pose.getX(DistanceUnit.INCH);
            double currentY = pose.getY(DistanceUnit.INCH);
            double currentHeading = pose.getHeading(AngleUnit.RADIANS);

            // Get target constraints from our mathematical splines
            double targetX = xSpline.getPosition(t, T);
            double targetY = ySpline.getPosition(t, T);

            // Feedforward velocity (What the spline naturally demands)
            double ffVx = xSpline.getVelocity(t, T);
            double ffVy = ySpline.getVelocity(t, T);

            // Calculate positional error
            double errorX = targetX - currentX;
            double errorY = targetY - currentY;

            // Compute global target velocity (Feedforward + P Controller)
            double globalVx = ffVx + (Kp_xy * errorX);
            double globalVy = ffVy + (Kp_xy * errorY);

            // Calculate heading error (We want the robot to face the direction of travel)
            // If you want fixed heading, change targetHeading to 0.
            double targetHeading = Math.atan2(ffVy, ffVx);
            if(ffVx == 0 && ffVy == 0) targetHeading = currentHeading; // Don't snap to 0 if stopped

            // Normalize heading error to [-pi, pi]
            double headingError = AngleUnit.normalizeRadians(targetHeading - currentHeading);
            double turnPower = headingError * Kp_heading;

            // Convert Global Velocity to Robot-Centric Velocity
            // We rotate the global velocity vector backwards by the robot's heading
            double robotVx = globalVx * Math.cos(-currentHeading) - globalVy * Math.sin(-currentHeading);
            double robotVy = globalVx * Math.sin(-currentHeading) + globalVy * Math.cos(-currentHeading);

            // Convert Robot-Centric velocities to Mecanum Wheel Powers
            // Standard FTC Convention: Y is forward, X is right, Turn is Counter-Clockwise
            double frontLeftPower  = robotVy + robotVx - turnPower;
            double frontRightPower = robotVy - robotVx + turnPower;
            double backLeftPower   = robotVy - robotVx - turnPower;
            double backRightPower  = robotVy + robotVx + turnPower;

            // Normalize powers so we don't exceed 1.0 or -1.0
            double max = Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower));
            max = Math.max(max, Math.abs(backLeftPower));
            max = Math.max(max, Math.abs(backRightPower));
            if (max > 1.0) {
                frontLeftPower /= max;
                frontRightPower /= max;
                backLeftPower /= max;
                backRightPower /= max;
            }

            setMotorPowers(frontLeftPower, frontRightPower, backLeftPower, backRightPower);

            // Telemetry for debugging
            telemetry.addData("Time", "%.2f / %.2f", t, T);
            telemetry.addData("Target (X,Y)", "%.1f, %.1f", targetX, targetY);
            telemetry.addData("Actual (X,Y)", "%.1f, %.1f", currentX, currentY);
            telemetry.addData("Errors (X,Y)", "%.1f, %.1f", errorX, errorY);
            telemetry.update();
        }
    }

    private void setMotorPowers(double fl, double fr, double bl, double br) {
        frontLeft.setPower(fl);
        frontRight.setPower(fr);
        backLeft.setPower(bl);
        backRight.setPower(br);
    }

    // ==========================================================
    // INNER CLASSES FOR QUINTIC MATH
    // ==========================================================

    /**
     * Represents the 1D state of the robot at a given point.
     */
    public static class State {
        public double p, v, a;
        public State(double position, double velocity, double acceleration) {
            this.p = position;
            this.v = velocity;
            this.a = acceleration;
        }
    }

    /**
     * Calculates the coefficients of a Quintic Polynomial based on Start and End constraints.
     */
    public static class QuinticPolynomial {
        private double a0, a1, a2, a3, a4, a5;

        public QuinticPolynomial(State start, State end, double T) {
            // Scale velocity and acceleration into the normalized [0, 1] time scale (tau)
            double v0 = start.v * T;
            double a0 = start.a * T * T;
            double v1 = end.v * T;
            double a1 = end.a * T * T;

            double p0 = start.p;
            double p1 = end.p;

            // Solve matrix coefficients
            this.a0 = p0;
            this.a1 = v0;
            this.a2 = 0.5 * a0;
            this.a3 = -10 * p0 - 6 * v0 - 1.5 * a0 + 10 * p1 - 4 * v1 + 0.5 * a1;
            this.a4 =  15 * p0 + 8 * v0 + 1.5 * a0 - 15 * p1 + 7 * v1 - a1;
            this.a5 =  -6 * p0 - 3 * v0 - 0.5 * a0 +  6 * p1 - 3 * v1 + 0.5 * a1;
        }

        public double getPosition(double t, double T) {
            double tau = t / T; // Normalize time [0,1]
            return a0 + a1*tau + a2*tau*tau + a3*Math.pow(tau, 3) + a4*Math.pow(tau, 4) + a5*Math.pow(tau, 5);
        }

        public double getVelocity(double t, double T) {
            double tau = t / T;
            double dTau_dt = 1.0 / T; // Chain rule derivative
            return (a1 + 2*a2*tau + 3*a3*tau*tau + 4*a4*Math.pow(tau, 3) + 5*a5*Math.pow(tau, 4)) * dTau_dt;
        }
    }
}