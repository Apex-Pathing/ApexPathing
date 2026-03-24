package com.apexpathing.drivetrain;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import org.jetbrains.annotations.NotNull;

/**
 * Drivetrain base class extended by MecanumDrive, TankDrive and SwerveDrive
 * @author Krish Joshi - 26192 Heatwaves
 * @author Sohum Arora 22985
 */
public abstract class Drivetrain {
    Telemetry telemetry;
    Boolean useBrakeMode;

    public HardwareMap hardwareMap;

    public Drivetrain() {
        this.telemetry = null;
    }
    public Drivetrain(Telemetry telemetry) {
        this.telemetry = telemetry;
    }


    public Drivetrain(Telemetry telemetry, boolean useBrakeMode) {
        this.telemetry = null;
        this.useBrakeMode = useBrakeMode;
    }

    public abstract void initDriveTrain(HardwareMap hardwareMap);

    public abstract void drive(double ...args);

    public abstract void turn(double power);

    public void setPower(DcMotorEx motor, double power) {
        motor.setPower(power);
    }
}
