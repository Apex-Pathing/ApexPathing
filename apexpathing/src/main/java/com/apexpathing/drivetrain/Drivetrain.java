package com.apexpathing.drivetrain;

import com.apexpathing.localization.Localizer;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Drivetrain base class extended by MecanumDrive, TankDrive and SwerveDrive
 * @author Krish Joshi - 26192 Heatwaves
 * @author Sohum Arora 22985
 */
public abstract class Drivetrain {
    Telemetry telemetry;
    Boolean useBrakeMode;

    public HardwareMap hardwareMap;
    private Localizer localizer;

    public Drivetrain() {
        this.telemetry = null;
    }
    public Drivetrain(HardwareMap hardwareMap,
                      Telemetry telemetry,
                      Localizer localizer) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        this.localizer = localizer;
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

    public final void update() {
        localizer.update();
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
