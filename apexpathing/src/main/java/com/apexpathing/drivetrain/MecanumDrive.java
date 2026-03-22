package com.apexpathing.drivetrain;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.apexpathing.geometry.Vector;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.jetbrains.annotations.NotNull;

import java.util.Arrays;
import java.util.List;

/**
 * A class that defines the Mecanum Drivetrain: a child class of the Drivetrain
 * @author Sohum Arora 22985
 * @author Krish 26192
 * @author Xander Haemel- 31616
 */
public class MecanumDrive extends Drivetrain {

    private final List<DcMotorEx> motors;
    private final VoltageSensor voltageSensor;
    private final double[] lastMotorPowers;

    private double motorCachingThreshold;
    private boolean useBrakeModeInTeleOp;
    private double staticFrictionCoefficient;
    private double nominalVoltage;
    private boolean voltageCompensation;
    private double maxPowerScaling = 1.0;

    private Vector[] vectors = new Vector[4];

    MecanumConstants constants = new MecanumConstants();

    //default constructor
    public MecanumDrive(HardwareMap hardwareMap, Telemetry telemetry, MecanumConstants mecanumConstants, @NotNull String leftFrontMotorName, @NotNull String rightFrontMotorName, @NotNull String leftRearMotorName, @NotNull String rightRearMotorName) {
        super(hardwareMap, telemetry, mecanumConstants.useBrakeModeInTeleOp, leftFrontMotorName, rightFrontMotorName, leftRearMotorName, rightRearMotorName);

        this.motors = Arrays.asList(leftFront, leftRear, rightFront, rightRear);
        this.lastMotorPowers = new double[]{0, 0, 0, 0};

        this.voltageSensor = hardwareMap.voltageSensor.iterator().next();

        this.motorCachingThreshold = mecanumConstants.motorCachingThreshold;
        this.useBrakeModeInTeleOp = mecanumConstants.useBrakeModeInTeleOp;
        this.staticFrictionCoefficient = mecanumConstants.staticFrictionCoefficient;
        this.nominalVoltage = mecanumConstants.nominalVoltage;
        this.voltageCompensation = mecanumConstants.useVoltageCompensation;

        for (DcMotorEx motor : motors) {
            MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
            motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
            motor.setMotorType(motorConfigurationType);
        }

        setMotorsToFloat();
        breakFollowing();

        Vector copiedFrontLeftVector = mecanumConstants.frontLeftVector.normalize();
        vectors = new Vector[]{
                new Vector(copiedFrontLeftVector.getMagnitude(), copiedFrontLeftVector.getTheta()),
                new Vector(copiedFrontLeftVector.getMagnitude(), 2 * Math.PI - copiedFrontLeftVector.getTheta()),
                new Vector(copiedFrontLeftVector.getMagnitude(), 2 * Math.PI - copiedFrontLeftVector.getTheta()),
                new Vector(copiedFrontLeftVector.getMagnitude(), copiedFrontLeftVector.getTheta())};
    }

    @Override
    public void driveBasedOnInputs(double x, double y, double turn) {
        botCentricDrive(x, y, turn);
    }

    @Override
    public void driveFieldCentric(double x, double y, double turn, double heading) {
        fieldCentricDrive(x, y, turn, heading);
    }

    public void updateConstants() {
        leftFront.setDirection(constants.leftFrontMotorDirection);
        leftRear.setDirection(constants.leftRearMotorDirection);
        rightFront.setDirection(constants.rightFrontMotorDirection);
        rightRear.setDirection(constants.rightRearMotorDirection);
        this.motorCachingThreshold = constants.motorCachingThreshold;
        this.useBrakeModeInTeleOp = constants.useBrakeModeInTeleOp;
        this.voltageCompensation = constants.useVoltageCompensation;
        this.nominalVoltage = constants.nominalVoltage;
        this.staticFrictionCoefficient = constants.staticFrictionCoefficient;
    }

    private void setMotorsToBrake() {
        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    private void setMotorsToFloat() {
        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    public void breakFollowing() {
        for (int i = 0; i < motors.size(); i++) {
            lastMotorPowers[i] = 0;
        }
        cutMotorPower();
        setMotorsToFloat();
    }

    public void runDrive(double[] drivePowers) {
        for (int i = 0; i < motors.size(); i++) {
            if (Math.abs(lastMotorPowers[i] - drivePowers[i]) > motorCachingThreshold ||
                    (drivePowers[i] == 0 && lastMotorPowers[i] != 0)) {
                lastMotorPowers[i] = drivePowers[i];
                setPower(motors.get(i), drivePowers[i]);
            }
        }
    }

    public void startTeleopDrive() {
        if (useBrakeModeInTeleOp) {
            setMotorsToBrake();
        }
    }

    public void startTeleopDrive(boolean brakeMode) {
        if (brakeMode) {
            setMotorsToBrake();
        } else {
            setMotorsToFloat();
        }
    }

    public void fieldCentricDrive(double x, double y, double turn, double robotHeading) {
        double cos = Math.cos(-robotHeading);
        double sin = Math.sin(-robotHeading);
        double fieldX = x * cos - y * sin;
        double fieldY = x * sin + y * cos;

        fieldX = deadzone(fieldX, 0.05);
        fieldY = deadzone(fieldY, 0.05);
        turn  = deadzone(turn, 0.05);

        double[] powers = new double[]{
                fieldY + fieldX + turn,
                fieldY - fieldX + turn,
                fieldY - fieldX - turn,
                fieldY + fieldX - turn
        };

        double max = Math.max(Math.max(Math.abs(powers[0]), Math.abs(powers[1])),
                Math.max(Math.abs(powers[2]), Math.abs(powers[3])));
        if (max > maxPowerScaling) {
            for (int i = 0; i < 4; i++) powers[i] = (powers[i] / max) * maxPowerScaling;
        }

        runDrive(powers);
    }

    public void botCentricDrive(double x, double y, double turn) {
        double adjX = deadzone(x, 0.05);
        double adjY = deadzone(y, 0.05);
        turn = deadzone(turn, 0.05);

        double[] powers = new double[]{
                adjY + adjX + turn,
                adjY - adjX + turn,
                adjY - adjX - turn,
                adjY + adjX - turn
        };

        double max = Math.max(Math.max(Math.abs(powers[0]), Math.abs(powers[1])),
                Math.max(Math.abs(powers[2]), Math.abs(powers[3])));
        if (max > maxPowerScaling) {
            for (int i = 0; i < 4; i++) powers[i] = (powers[i] / max) * maxPowerScaling;
        }

        runDrive(powers);
    }

    private static double deadzone(double value, double threshold) {
        return Math.abs(value) < threshold ? 0.0 : value;
    }

    public void getAndRunDrivePowers(Vector correctivePower, Vector headingPower, Vector pathingPower, double robotHeading) {
        runDrive(calculateDrive(correctivePower, headingPower, pathingPower, robotHeading));
    }

    public double xVelocity() { return constants.xVelocity; }
    public double yVelocity() { return constants.yVelocity; }
    public void setXVelocity(double xMovement) { constants.setXVelocity(xMovement); }
    public void setYVelocity(double yMovement) { constants.setYVelocity(yMovement); }
    public double getStaticFrictionCoefficient() { return staticFrictionCoefficient; }

    public double getVoltage() {
        return voltageSensor.getVoltage();
    }

    private double getVoltageNormalized() {
        double voltage = getVoltage();
        return (nominalVoltage - (nominalVoltage * staticFrictionCoefficient)) /
                (voltage - ((nominalVoltage * nominalVoltage / voltage) * staticFrictionCoefficient));
    }

    public List<DcMotorEx> getMotors() { return motors; }

    public double[] calculateDrive(Vector correctivePower, Vector headingPower, Vector pathingPower, double robotHeading) {
        if (correctivePower.getMagnitude() > maxPowerScaling)
            correctivePower.setMagnitude(maxPowerScaling);
        if (headingPower.getMagnitude() > maxPowerScaling)
            headingPower.setMagnitude(maxPowerScaling);
        if (pathingPower.getMagnitude() > maxPowerScaling)
            pathingPower.setMagnitude(maxPowerScaling);

        double[] wheelPowers = new double[4];
        Vector[] mecanumVectorsCopy = new Vector[4];
        Vector[] truePathingVectors = new Vector[2];

        if (correctivePower.getMagnitude() == maxPowerScaling) {
            truePathingVectors[0] = correctivePower.copy();
            truePathingVectors[1] = correctivePower.copy();
        } else {
            Vector leftSideVector = correctivePower.minus(headingPower);
            Vector rightSideVector = correctivePower.plus(headingPower);

            if (leftSideVector.getMagnitude() > maxPowerScaling || rightSideVector.getMagnitude() > maxPowerScaling) {
                double headingScalingFactor = Math.min(findNormalizingScaling(correctivePower, headingPower, maxPowerScaling), findNormalizingScaling(correctivePower, headingPower.times(-1), maxPowerScaling));
                truePathingVectors[0] = correctivePower.minus(headingPower.times(headingScalingFactor));
                truePathingVectors[1] = correctivePower.plus(headingPower.times(headingScalingFactor));
            } else {
                Vector leftSideVectorWithPathing = leftSideVector.plus(pathingPower);
                Vector rightSideVectorWithPathing = rightSideVector.plus(pathingPower);

                if (leftSideVectorWithPathing.getMagnitude() > maxPowerScaling || rightSideVectorWithPathing.getMagnitude() > maxPowerScaling) {
                    double pathingScalingFactor = Math.min(findNormalizingScaling(leftSideVector, pathingPower, maxPowerScaling), findNormalizingScaling(rightSideVector, pathingPower, maxPowerScaling));
                    truePathingVectors[0] = leftSideVector.plus(pathingPower.times(pathingScalingFactor));
                    truePathingVectors[1] = rightSideVector.plus(pathingPower.times(pathingScalingFactor));
                } else {
                    truePathingVectors[0] = leftSideVectorWithPathing.copy();
                    truePathingVectors[1] = rightSideVectorWithPathing.copy();
                }
            }
        }

        truePathingVectors[0] = truePathingVectors[0].times(2.0);
        truePathingVectors[1] = truePathingVectors[1].times(2.0);

        for (int i = 0; i < mecanumVectorsCopy.length; i++) {
            mecanumVectorsCopy[i] = vectors[i].copy();
            mecanumVectorsCopy[i].rotateVector(robotHeading);
        }

        wheelPowers[0] = (mecanumVectorsCopy[1].getXComponent() * truePathingVectors[0].getYComponent() - truePathingVectors[0].getXComponent() * mecanumVectorsCopy[1].getYComponent()) / (mecanumVectorsCopy[1].getXComponent() * mecanumVectorsCopy[0].getYComponent() - mecanumVectorsCopy[0].getXComponent() * mecanumVectorsCopy[1].getYComponent());
        wheelPowers[1] = (mecanumVectorsCopy[0].getXComponent() * truePathingVectors[0].getYComponent() - truePathingVectors[0].getXComponent() * mecanumVectorsCopy[0].getYComponent()) / (mecanumVectorsCopy[0].getXComponent() * mecanumVectorsCopy[1].getYComponent() - mecanumVectorsCopy[1].getXComponent() * mecanumVectorsCopy[0].getYComponent());
        wheelPowers[2] = (mecanumVectorsCopy[3].getXComponent() * truePathingVectors[1].getYComponent() - truePathingVectors[1].getXComponent() * mecanumVectorsCopy[3].getYComponent()) / (mecanumVectorsCopy[3].getXComponent() * mecanumVectorsCopy[2].getYComponent() - mecanumVectorsCopy[2].getXComponent() * mecanumVectorsCopy[3].getYComponent());
        wheelPowers[3] = (mecanumVectorsCopy[2].getXComponent() * truePathingVectors[1].getYComponent() - truePathingVectors[1].getXComponent() * mecanumVectorsCopy[2].getYComponent()) / (mecanumVectorsCopy[2].getXComponent() * mecanumVectorsCopy[3].getYComponent() - mecanumVectorsCopy[3].getXComponent() * mecanumVectorsCopy[2].getYComponent());

        if (voltageCompensation) {
            double voltageNormalized = getVoltageNormalized();
            for (int i = 0; i < wheelPowers.length; i++) {
                wheelPowers[i] *= voltageNormalized;
            }
        }

        double max = Math.max(Math.max(Math.abs(wheelPowers[0]), Math.abs(wheelPowers[1])),
                Math.max(Math.abs(wheelPowers[2]), Math.abs(wheelPowers[3])));

        if (max > maxPowerScaling) {
            for (int i = 0; i < 4; i++) {
                wheelPowers[i] = (wheelPowers[i] / max) * maxPowerScaling;
            }
        }

        return wheelPowers;
    }

    private double findNormalizingScaling(Vector base, Vector addition, double maxMagnitude) {
        double a = addition.getXComponent() * addition.getXComponent() + addition.getYComponent() * addition.getYComponent();
        double b = 2 * (base.getXComponent() * addition.getXComponent() + base.getYComponent() * addition.getYComponent());
        double c = base.getXComponent() * base.getXComponent() + base.getYComponent() * base.getYComponent() - maxMagnitude * maxMagnitude;
        double discriminant = b * b - 4 * a * c;
        return (-b + Math.sqrt(discriminant)) / (2 * a);
    }
}
