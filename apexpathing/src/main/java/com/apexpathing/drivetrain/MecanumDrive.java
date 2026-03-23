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
 * @author Krish Joshi - 26192 Heatwaves
 */
public class MecanumDrive extends Drivetrain {

    private List<DcMotorEx> motors;
    private double maxPowerScaling = 1.0;

    private DcMotorEx leftFront, leftRear, rightFront, rightRear;
    private final String leftFrontMotorName, leftRearMotorName, rightFrontMotorName, rightRearMotorName;

    private final HardwareMap hardwareMap;
    private final MecanumConstants constants;

    public MecanumDrive(HardwareMap hardwareMap, MecanumConstants constants) {
        this.hardwareMap = hardwareMap;
        this.constants = constants;
        this.leftFrontMotorName=constants.leftFrontMotorName;
        this.leftRearMotorName=constants.leftRearMotorName;
        this.rightFrontMotorName=constants.rightFrontMotorName;
        this.rightRearMotorName=constants.rightRearMotorName;
    }

    private void initDriveTrain(MecanumConstants constants) {

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

        for(int i = 0; i < 4; i++) {
            motors.get(i).setPower(powers[i]);
        }
    }

    @Override
    public void initDriveTrain() {
        leftFront = (DcMotorEx)hardwareMap.get(DcMotor.class, leftFrontMotorName);
        leftRear = (DcMotorEx)hardwareMap.get(DcMotor.class, leftRearMotorName);
        rightFront = (DcMotorEx)hardwareMap.get(DcMotor.class, rightFrontMotorName);
        rightRear = (DcMotorEx)hardwareMap.get(DcMotor.class, rightRearMotorName);

        leftFront.setDirection(constants.leftFrontMotorDirection);
        leftRear.setDirection(constants.leftRearMotorDirection);
        rightFront.setDirection(constants.rightFrontMotorDirection);
        rightRear.setDirection(constants.rightRearMotorDirection);

        this.motors = Arrays.asList(leftFront, leftRear, rightFront, rightRear);

        for (DcMotorEx motor : motors) {
            DcMotor.ZeroPowerBehavior brake = DcMotor.ZeroPowerBehavior.FLOAT;
            if (constants.useBrakeMode) {
                brake = DcMotor.ZeroPowerBehavior.BRAKE;
            }
            motor.setZeroPowerBehavior(brake);
        }
    }

    @Override
    public void drive(double... args) {
        botCentricDrive(args[0], args[1], args[2]);
    }

    @Override
    public void turn(double power) {
        botCentricDrive(0.0,0.0,power);
    }
}
