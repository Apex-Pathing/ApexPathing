package com.apexpathing.drivetrain;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.jetbrains.annotations.NotNull;

public class TankDrive extends Drivetrain {

    boolean fourWheelDrive;

    public TankDrive(HardwareMap hardwareMap, Telemetry telemetry,
                     boolean useBrakeMode,
                     @NotNull String leftFrontName,
                     @NotNull String rightFrontName) {
        super(hardwareMap, telemetry, useBrakeMode, leftFrontName, rightFrontName, leftFrontName, rightFrontName);
        this.fourWheelDrive = false;
    }

    public TankDrive(HardwareMap hardwareMap,
                     Telemetry telemetry,
                     boolean useBrakeMode,
                     @NotNull String leftFrontName,
                     @NotNull String rightFrontName,
                     @NotNull String leftRearName,
                     @NotNull String rightRearName) {
        super(hardwareMap, telemetry, useBrakeMode, leftFrontName, rightFrontName, leftRearName, rightRearName);
        this.fourWheelDrive = true;
    }

    @Override
    public void drive(double x, double y, double turn) {
        double left  = Range.clip(y + turn, -1, 1);
        double right = Range.clip(y - turn, -1, 1);

        if (!fourWheelDrive) {
            setPower(leftFront, left);
            setPower(rightFront, right);
        } else {
            setPower(leftFront, left);
            setPower(rightFront, right);
            setPower(leftRear, left);
            setPower(rightRear, right);
        }
    }

    @Override
    public void driveFieldCentric(double x, double y, double turn, double heading) {
        // Field-centric not applicable to tank drive
    }
}
