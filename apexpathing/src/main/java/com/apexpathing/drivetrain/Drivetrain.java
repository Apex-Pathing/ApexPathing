package com.apexpathing.drivetrain;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import org.jetbrains.annotations.NotNull;

public abstract class Drivetrain {
    public DcMotorEx leftFront, leftRear, rightFront, rightRear;
    Telemetry telemetry;
    public Servo leftFrontServo, leftRearServo, rightFrontServo, rightRearServo;

    public Drivetrain(HardwareMap hardwareMap,
                      Telemetry telemetry,
                      boolean useBrakeMode,
                      @NotNull String leftFrontName,
                      @NotNull String rightFrontName,
                      @NotNull String leftRearName,
                      @NotNull String rightRearName) {
        this.telemetry = telemetry;

        leftFront = hardwareMap.get(DcMotorEx.class, leftFrontName);
        leftRear = hardwareMap.get(DcMotorEx.class, leftRearName);
        rightFront = hardwareMap.get(DcMotorEx.class, rightFrontName);
        rightRear = hardwareMap.get(DcMotorEx.class, rightRearName);

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightRear.setDirection(DcMotorSimple.Direction.FORWARD);

        setMotorRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        if (useBrakeMode) {
            setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        } else {
            setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }
    }

    public Drivetrain(HardwareMap hardwareMap,
                      Telemetry telemetry,
                      boolean useBrakeMode,
                      @NotNull String leftFrontName,
                      @NotNull String rightFrontName,
                      @NotNull String leftRearName,
                      @NotNull String rightRearName,
                      String leftFrontServoName,
                      String rightFrontServoName,
                      String leftRearServoName,
                      String rightRearServoName) {
        this(hardwareMap, telemetry, useBrakeMode, leftFrontName, rightFrontName, leftRearName, rightRearName);

        leftFrontServo = hardwareMap.get(Servo.class, leftFrontServoName);
        rightFrontServo = hardwareMap.get(Servo.class, rightFrontServoName);
        leftRearServo = hardwareMap.get(Servo.class, leftRearServoName);
        rightRearServo = hardwareMap.get(Servo.class, rightRearServoName);
    }

    public abstract void driveBasedOnInputs(double x, double y, double turn);

    public abstract void driveFieldCentric(double x, double y, double turn, double heading);

    public void cutMotorPower() {
        setPower(0);
    }

    public void setMotorRunMode(DcMotor.RunMode mode) {
        leftFront.setMode(mode);
        leftRear.setMode(mode);
        rightFront.setMode(mode);
        rightRear.setMode(mode);
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior) {
        leftFront.setZeroPowerBehavior(behavior);
        leftRear.setZeroPowerBehavior(behavior);
        rightFront.setZeroPowerBehavior(behavior);
        rightRear.setZeroPowerBehavior(behavior);
    }

    public double[] getMotorPowers() {
        return new double[]{
                leftFront.getPower(),
                leftRear.getPower(),
                rightFront.getPower(),
                rightRear.getPower()
        };
    }

    public double[] getMotorPositions() {
        return new double[]{
                leftFront.getCurrentPosition(),
                leftRear.getCurrentPosition(),
                rightFront.getCurrentPosition(),
                rightRear.getCurrentPosition()
        };
    }

    public void setPower(double power) {
        leftFront.setPower(power);
        leftRear.setPower(power);
        rightFront.setPower(power);
        rightRear.setPower(power);
    }

    public void setPower(DcMotorEx motor, double power) {
        motor.setPower(power);
    }

    public void setServo(Servo servo, double pos) {
        servo.setPosition(pos);
    }

    public void turn(double power) {
        leftFront.setPower(-power);
        leftRear.setPower(-power);
        rightFront.setPower(power);
        rightRear.setPower(power);
    }

    public void resetEncoders() {
        setMotorRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setMotorRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void logMotors() {
        telemetry.addLine("---Power---");
        telemetry.addData("leftFront Power", leftFront.getPower());
        telemetry.addData("rightFront Power", rightFront.getPower());
        telemetry.addData("leftRear Power", leftRear.getPower());
        telemetry.addData("rightRear Power", rightRear.getPower());

        telemetry.addLine("---Velocity---");
        telemetry.addData("leftFront velocity", leftFront.getVelocity());
        telemetry.addData("rightFront velocity", rightFront.getVelocity());
        telemetry.addData("leftRear velocity", leftRear.getVelocity());
        telemetry.addData("rightRear velocity", rightRear.getVelocity());
    }

    public void logServos() {
        telemetry.addData("leftFrontServo pos", leftFrontServo.getPosition());
        telemetry.addData("rightFrontServo pos", rightFrontServo.getPosition());
        telemetry.addData("leftRearServo pos", leftRearServo.getPosition());
        telemetry.addData("rightRearServo pos", rightRearServo.getPosition());
    }
}
