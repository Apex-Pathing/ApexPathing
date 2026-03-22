package com.apexpathing.drivetrain;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public abstract class Drivetrain {
    Telemetry telemetry;
    Boolean useBrakeMode;

    public HardwareMap hardwareMap;

    public Drivetrain() {
        this.telemetry = null;
    }
    public Drivetrain(HardwareMap hardwareMap,
                      Telemetry telemetry) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
    }

    public Drivetrain(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
        this.telemetry = null;
    }

    public Drivetrain(HardwareMap hardwareMap, Telemetry telemetry, boolean useBrakeMode) {
        this.hardwareMap = hardwareMap;
        this.telemetry = null;
        this.useBrakeMode = useBrakeMode;
    }

    public abstract void initDriveTrain();

    public void setHardwareMap(HardwareMap hw) {
        this.hardwareMap = hw;
    }

    public abstract void drive(double ...args);

    public abstract void turn(double power);

    public void setPower(DcMotorEx motor, double power) {
        motor.setPower(power);
    }
}