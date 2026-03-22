package com.apexpathing.drivetrain;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.apexpathing.util.math.Pose;
import com.apexpathing.util.math.Vector;
import com.apexpathing.localization.Localizer;
import com.apexpathing.follower.HolonomicTrajectoryFollower;

import org.jetbrains.annotations.NotNull;

import java.util.Arrays;
import java.util.List;

/**
 * A class that defines the Mecanum Drivetrain.
 * Now extends CustomDrive for centralized update logic.
 * @author Krish Joshi - 26192 Heatwaves
 */
public class MecanumDrive extends CustomDrive {

    private List<DcMotorEx> motors;
    private double[] lastMotorPowers;

    private double motorCachingThreshold;
    private double maxPowerScaling = 1.0;

    private DcMotorEx leftFront, leftRear, rightFront, rightRear;
    private final String leftFrontMotorName, leftRearMotorName, rightFrontMotorName, rightRearMotorName;
    private final HardwareMap hardwareMap;

    /**
     * @param hardwareMap      this is the HardwareMap object that contains the motors and other hardware
     * @param localizer        the localizer used for position tracking
     * @param mecanumConstants this is the MecanumConstants object that contains the names of the motors and directions etc.
     */
    public MecanumDrive(HardwareMap hardwareMap, Localizer localizer, MecanumConstants mecanumConstants, @NotNull String leftFrontMotorName, @NotNull String rightFrontMotorName, @NotNull String leftRearMotorName, @NotNull String rightRearMotorName) {
        this.hardwareMap = hardwareMap;
        this.localizer = localizer;
        this.leftFrontMotorName = leftFrontMotorName;
        this.rightFrontMotorName = rightFrontMotorName;
        this.leftRearMotorName = leftRearMotorName;
        this.rightRearMotorName = rightRearMotorName;
        this.controller = new HolonomicTrajectoryFollower();

        initDriveTrain(mecanumConstants);
    }

    private void initDriveTrain(MecanumConstants constants) {
        leftFront = (DcMotorEx)hardwareMap.get(DcMotor.class, leftFrontMotorName);
        leftRear = (DcMotorEx)hardwareMap.get(DcMotor.class, leftRearMotorName);
        rightFront = (DcMotorEx)hardwareMap.get(DcMotor.class, rightFrontMotorName);
        rightRear = (DcMotorEx)hardwareMap.get(DcMotor.class, rightRearMotorName);

        leftFront.setDirection(constants.leftFrontMotorDirection);
        leftRear.setDirection(constants.leftRearMotorDirection);
        rightFront.setDirection(constants.rightFrontMotorDirection);
        rightRear.setDirection(constants.rightRearMotorDirection);

        this.motors = Arrays.asList(leftFront, leftRear, rightFront, rightRear);
        this.lastMotorPowers = new double[]{0, 0, 0, 0};

        this.voltageSensor = hardwareMap.voltageSensor.iterator().next();
        this.motorCachingThreshold = constants.motorCachingThreshold;
        this.nominalVoltage = constants.nominalVoltage;
        this.useVoltageCompensation = constants.useVoltageCompensation;
        this.staticFrictionCoefficient = constants.staticFrictionCoefficient;

        for (DcMotorEx motor : motors) {
            MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
            motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
            motor.setMotorType(motorConfigurationType);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }
    }

    @Override
    public void setDrivePowers(Pose drivePowers) {
        botCentricDrive(drivePowers.x(), drivePowers.y(), drivePowers.heading());
    }

    public void botCentricDrive(double x, double y, double turn) {
        double[] powers = new double[]{
                y + x + turn,  // left front
                y - x + turn,  // left rear
                y - x - turn,  // right front
                y + x - turn   // Right rear
        };

        double max = Math.max(Math.max(Math.abs(powers[0]), Math.abs(powers[1])),
                Math.max(Math.abs(powers[2]), Math.abs(powers[3])));
        if (max > maxPowerScaling) {
            for (int i = 0; i < 4; i++) powers[i] = (powers[i] / max) * maxPowerScaling;
        }

        runDrive(powers);
    }

    private void runDrive(double[] drivePowers) {
        for (int i = 0; i < motors.size(); i++) {
            if (Math.abs(lastMotorPowers[i] - drivePowers[i]) > motorCachingThreshold ||
                    (drivePowers[i] == 0 && lastMotorPowers[i] != 0)) {
                lastMotorPowers[i] = drivePowers[i];
                motors.get(i).setPower(drivePowers[i]);
            }
        }
    }
}
