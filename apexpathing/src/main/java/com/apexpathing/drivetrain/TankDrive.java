package com.apexpathing.drivetrain;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.jetbrains.annotations.NotNull;

/**
 * @author Sohum Arora 22985 Paraducks
 */
public class TankDrive extends Drivetrain {

    private final boolean fourWheelDrive;

    // 4wd tank constructor
    public TankDrive(HardwareMap hardwareMap, Telemetry telemetry,
                     boolean useBrakeMode,
                     @NotNull String leftFrontName,
                     @NotNull String rightFrontName) {
        super(hardwareMap, telemetry, useBrakeMode,
                leftFrontName, rightFrontName, leftFrontName, rightFrontName);
        this.fourWheelDrive = false;
    }

    // 4wd tank constructor
    public TankDrive(HardwareMap hardwareMap, Telemetry telemetry,
                     boolean useBrakeMode,
                     @NotNull String leftFrontName,
                     @NotNull String rightFrontName,
                     @NotNull String leftRearName,
                     @NotNull String rightRearName) {
        super(hardwareMap, telemetry, useBrakeMode,
                leftFrontName, rightFrontName, leftRearName, rightRearName);
        this.fourWheelDrive = true;
    }

    @Override
    public void drive(double x, double y, double turn) {
        double left  = Range.clip(y + turn, -1, 1);
        double right = Range.clip(y - turn, -1, 1);

        setPower(leftFront, left);
        setPower(rightFront, right);

        if (fourWheelDrive) {
            setPower(leftRear, left);
            setPower(rightRear, right);
        }
    }

    @Override
    public void driveFieldCentric(double x, double y, double turn, double heading) {
        //Tank has no field centric option
        drive(x, y, turn);
    }

    @Override
    public void turn(double power) {
        drive(0, 0, power);
    }
}