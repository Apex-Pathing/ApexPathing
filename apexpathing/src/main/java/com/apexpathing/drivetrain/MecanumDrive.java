package com.apexpathing.drivetrain;

import com.apexpathing.util.math.Vector;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.jetbrains.annotations.NotNull;

import java.util.Arrays;
import java.util.List;

/**
 * Mecanum drivetrain implementation with vector-based drive calculation,
 * motor caching, and optional voltage compensation.
 *
 * @author Sohum Arora 22985 Paraducks
 */
public class MecanumDrive extends Drivetrain {

    // -------------------------------------------------------------------------
    // Fields
    // -------------------------------------------------------------------------

    private final List<DcMotorEx> motors;
    private final VoltageSensor voltageSensor;
    private final double[] lastMotorPowers;
    private final Vector[] mecanumVectors;
    private final MecanumConstants constants;

    private double motorCachingThreshold;
    private boolean useBrakeModeInTeleOp;
    private double staticFrictionCoefficient;
    private double nominalVoltage;
    private boolean voltageCompensation;
    private double maxPowerScaling = 1.0;

    // -------------------------------------------------------------------------
    // Constructor
    // -------------------------------------------------------------------------

    public MecanumDrive(HardwareMap hardwareMap,
                        Telemetry telemetry,
                        MecanumConstants mecanumConstants,
                        @NotNull String leftFrontMotorName,
                        @NotNull String rightFrontMotorName,
                        @NotNull String leftRearMotorName,
                        @NotNull String rightRearMotorName) {
        super(hardwareMap, telemetry, mecanumConstants.useBrakeModeInTeleOp,
                leftFrontMotorName, rightFrontMotorName, leftRearMotorName, rightRearMotorName);

        this.constants = mecanumConstants;
        this.motors = Arrays.asList(leftFront, leftRear, rightFront, rightRear);
        this.lastMotorPowers = new double[]{0, 0, 0, 0};
        this.voltageSensor = hardwareMap.voltageSensor.iterator().next();

        this.motorCachingThreshold   = mecanumConstants.motorCachingThreshold;
        this.useBrakeModeInTeleOp    = mecanumConstants.useBrakeModeInTeleOp;
        this.staticFrictionCoefficient = mecanumConstants.staticFrictionCoefficient;
        this.nominalVoltage          = mecanumConstants.nominalVoltage;
        this.voltageCompensation     = mecanumConstants.useVoltageCompensation;

        // Set motors to max RPM fraction
        for (DcMotorEx motor : motors) {
            MotorConfigurationType type = motor.getMotorType().clone();
            type.setAchieveableMaxRPMFraction(1.0);
            motor.setMotorType(type);
        }

        // Build mecanum wheel vectors from the front-left vector in constants
        Vector fl = mecanumConstants.frontLeftVector.normalize();
        this.mecanumVectors = new Vector[]{
                new Vector(fl.getMagnitude(), fl.getTheta()),                        // left front
                new Vector(fl.getMagnitude(), 2 * Math.PI - fl.getTheta()),          // left rear
                new Vector(fl.getMagnitude(), 2 * Math.PI - fl.getTheta()),          // right front
                new Vector(fl.getMagnitude(), fl.getTheta())                         // right rear
        };

        setMotorsToFloat();
        breakFollowing();
    }

    // -------------------------------------------------------------------------
    // Abstract overrides
    // -------------------------------------------------------------------------

    /** Bot-centric drive. */
    @Override
    public void drive(double x, double y, double turn) {
        botCentricDrive(x, y, turn);
    }

    /** Field-centric drive. */
    @Override
    public void driveFieldCentric(double x, double y, double turn, double heading) {
        fieldCentricDrive(x, y, turn, heading);
    }

    // -------------------------------------------------------------------------
    // Drive methods
    // -------------------------------------------------------------------------

    /** Standard bot-centric mecanum drive. */
    public void botCentricDrive(double x, double y, double turn) {
        double adjX   = deadzone(x,    0.05);
        double adjY   = deadzone(y,    0.05);
        double adjTurn = deadzone(turn, 0.05);

        double[] powers = {
                adjY + adjX + adjTurn,   // left front
                adjY - adjX + adjTurn,   // left rear
                adjY - adjX - adjTurn,   // right front
                adjY + adjX - adjTurn    // right rear
        };

        normalizePowers(powers);
        runDrive(powers);
    }

    /** Field-centric mecanum drive — rotates input by robot heading before driving. */
    public void fieldCentricDrive(double x, double y, double turn, double robotHeading) {
        double cos    = Math.cos(-robotHeading);
        double sin    = Math.sin(-robotHeading);
        double fieldX = deadzone(x * cos - y * sin, 0.05);
        double fieldY = deadzone(x * sin + y * cos, 0.05);
        double adjTurn = deadzone(turn, 0.05);

        double[] powers = {
                fieldY + fieldX + adjTurn,  // left front
                fieldY - fieldX + adjTurn,  // left rear
                fieldY - fieldX - adjTurn,  // right front
                fieldY + fieldX - adjTurn   // right rear
        };

        normalizePowers(powers);
        runDrive(powers);
    }

    /**
     * Vector-based pathing drive used by Pedro Pathing / autonomous.
     * Combines corrective, heading, and pathing vectors into wheel powers.
     */
    public void getAndRunDrivePowers(Vector correctivePower,
                                     Vector headingPower,
                                     Vector pathingPower,
                                     double robotHeading) {
        runDrive(calculateDrive(correctivePower, headingPower, pathingPower, robotHeading));
    }

    // -------------------------------------------------------------------------
    // Core vector-based drive calculation
    // -------------------------------------------------------------------------

    public double[] calculateDrive(Vector correctivePower,
                                   Vector headingPower,
                                   Vector pathingPower,
                                   double robotHeading) {
        // Clamp input vectors
        clampVector(correctivePower);
        clampVector(headingPower);
        clampVector(pathingPower);

        // Resolve left/right side pathing vectors
        Vector[] sideVectors = resolveSideVectors(correctivePower, headingPower, pathingPower);
        sideVectors[0] = sideVectors[0].times(2.0);
        sideVectors[1] = sideVectors[1].times(2.0);

        // Rotate mecanum vectors by robot heading
        Vector[] rotatedMecanum = new Vector[4];
        for (int i = 0; i < 4; i++) {
            rotatedMecanum[i] = mecanumVectors[i].copy();
            rotatedMecanum[i].rotateVec(robotHeading);
        }

        // Project side vectors onto mecanum wheel axes
        double[] wheelPowers = new double[4];
        wheelPowers[0] = cross2D(rotatedMecanum[1], sideVectors[0]) / cross2D(rotatedMecanum[1], rotatedMecanum[0]);
        wheelPowers[1] = cross2D(rotatedMecanum[0], sideVectors[0]) / cross2D(rotatedMecanum[0], rotatedMecanum[1]);
        wheelPowers[2] = cross2D(rotatedMecanum[3], sideVectors[1]) / cross2D(rotatedMecanum[3], rotatedMecanum[2]);
        wheelPowers[3] = cross2D(rotatedMecanum[2], sideVectors[1]) / cross2D(rotatedMecanum[2], rotatedMecanum[3]);

        // Optional voltage compensation
        if (voltageCompensation) {
            double scale = getVoltageNormalized();
            for (int i = 0; i < 4; i++) wheelPowers[i] *= scale;
        }

        normalizePowers(wheelPowers);
        return wheelPowers;
    }

    /**
     * Resolves left and right side vectors from corrective, heading, and pathing vectors.
     * Scales heading/pathing down if they would exceed maxPowerScaling.
     */
    private Vector[] resolveSideVectors(Vector corrective, Vector heading, Vector pathing) {
        // If corrective already maxed, heading and pathing are dropped
        if (corrective.getMagnitude() >= maxPowerScaling) {
            return new Vector[]{ corrective.copy(), corrective.copy() };
        }

        Vector leftBase  = corrective.minus(heading);
        Vector rightBase = corrective.plus(heading);

        // Scale heading if left or right base exceeds max
        if (leftBase.getMagnitude() > maxPowerScaling || rightBase.getMagnitude() > maxPowerScaling) {
            double scale = Math.min(
                    findNormalizingScaling(corrective, heading, maxPowerScaling),
                    findNormalizingScaling(corrective, heading.times(-1), maxPowerScaling)
            );
            return new Vector[]{
                    corrective.minus(heading.times(scale)),
                    corrective.plus(heading.times(scale))
            };
        }

        Vector leftFinal  = leftBase.plus(pathing);
        Vector rightFinal = rightBase.plus(pathing);

        // Scale pathing if adding it exceeds max
        if (leftFinal.getMagnitude() > maxPowerScaling || rightFinal.getMagnitude() > maxPowerScaling) {
            double scale = Math.min(
                    findNormalizingScaling(leftBase, pathing, maxPowerScaling),
                    findNormalizingScaling(rightBase, pathing, maxPowerScaling)
            );
            return new Vector[]{
                    leftBase.plus(pathing.times(scale)),
                    rightBase.plus(pathing.times(scale))
            };
        }

        return new Vector[]{ leftFinal, rightFinal };
    }

    // -------------------------------------------------------------------------
    // Motor management
    // -------------------------------------------------------------------------

    /** Sets motor powers with caching — skips update if change is below threshold. */
    public void runDrive(double[] drivePowers) {
        for (int i = 0; i < motors.size(); i++) {
            boolean changed  = Math.abs(lastMotorPowers[i] - drivePowers[i]) > motorCachingThreshold;
            boolean zeroed   = drivePowers[i] == 0 && lastMotorPowers[i] != 0;
            if (changed || zeroed) {
                lastMotorPowers[i] = drivePowers[i];
                setPower(motors.get(i), drivePowers[i]);
            }
        }
    }

    /** Resets cached powers and stops motors. */
    public void breakFollowing() {
        for (int i = 0; i < motors.size(); i++) lastMotorPowers[i] = 0;
        setPower(0);
        setMotorsToFloat();
    }

    public void startTeleopDrive() {
        if (useBrakeModeInTeleOp) setMotorsToBrake();
        else setMotorsToFloat();
    }

    public void startTeleopDrive(boolean brakeMode) {
        if (brakeMode) setMotorsToBrake();
        else setMotorsToFloat();
    }

    private void setMotorsToBrake() {
        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    private void setMotorsToFloat() {
        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    /** Syncs live constants (useful with FTC Dashboard). */
    public void updateConstants() {
        leftFront.setDirection(constants.leftFrontMotorDirection);
        leftRear.setDirection(constants.leftRearMotorDirection);
        rightFront.setDirection(constants.rightFrontMotorDirection);
        rightRear.setDirection(constants.rightRearMotorDirection);
        this.motorCachingThreshold     = constants.motorCachingThreshold;
        this.useBrakeModeInTeleOp      = constants.useBrakeModeInTeleOp;
        this.voltageCompensation       = constants.useVoltageCompensation;
        this.nominalVoltage            = constants.nominalVoltage;
        this.staticFrictionCoefficient = constants.staticFrictionCoefficient;
    }

    // -------------------------------------------------------------------------
    // Getters / setters
    // -------------------------------------------------------------------------

    public double xVelocity() { return constants.xVelocity; }
    public double yVelocity()  { return constants.yVelocity; }
    public void setXVelocity(double v) { constants.setXVelocity(v); }
    public void setYVelocity(double v){ constants.setYVelocity(v); }
    public double getStaticFrictionCoefficient(){ return staticFrictionCoefficient; }
    public double getVoltage() { return voltageSensor.getVoltage(); }
    public List<DcMotorEx> getMotors() { return motors; }



    private void clampVector(Vector v) {
        if (v.getMagnitude() > maxPowerScaling) v.setMagnitude(maxPowerScaling);
    }

    private void normalizePowers(double[] powers) {
        double max = 0;
        for (double p : powers) max = Math.max(max, Math.abs(p));
        if (max > maxPowerScaling)
            for (int i = 0; i < powers.length; i++) powers[i] = (powers[i] / max) * maxPowerScaling;
    }

    private double getVoltageNormalized() {
        double v = getVoltage();
        return (nominalVoltage - nominalVoltage * staticFrictionCoefficient)
                / (v - (nominalVoltage * nominalVoltage / v) * staticFrictionCoefficient);
    }

    /** 2D cross product (scalar) of two vectors. */
    private double cross2D(Vector a, Vector b) {
        return a.getXComponent() * b.getYComponent() - b.getXComponent() * a.getYComponent();
    }

    /**
     * Finds the scalar t such that |base + t*addition| == maxMagnitude.
     * Used to scale heading/pathing vectors to fit within power limits.
     */
    private double findNormalizingScaling(Vector base, Vector addition, double maxMagnitude) {
        double a = addition.getXComponent() * addition.getXComponent()
                + addition.getYComponent() * addition.getYComponent();
        double b = 2 * (base.getXComponent() * addition.getXComponent()
                + base.getYComponent() * addition.getYComponent());
        double c = base.getXComponent() * base.getXComponent()
                + base.getYComponent() * base.getYComponent()
                - maxMagnitude * maxMagnitude;
        return (-b + Math.sqrt(b * b - 4 * a * c)) / (2 * a);
    }

    private static double deadzone(double value, double threshold) {
        return Math.abs(value) < threshold ? 0.0 : value;
    }
}