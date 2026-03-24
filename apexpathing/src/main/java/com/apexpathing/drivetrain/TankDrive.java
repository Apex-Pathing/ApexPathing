package com.apexpathing.drivetrain;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.jetbrains.annotations.NotNull;

/**
 * @author Krish Joshi - 26192 Heatwaves
 */
public class TankDrive extends Drivetrain {

    boolean FourWheelDrive;

    DcMotorEx leftFront, rightFront, leftRear, rightRear;
    String leftFrontName, rightFrontName, leftRearName, rightRearName;


    public TankDrive(Telemetry telemetry,
                     boolean useBrakeMode,
                     @NotNull String leftFrontName,
                     @NotNull String rightFrontName
                     ) { //2wd tank drive constructor
        super(telemetry, useBrakeMode);
        this.leftFrontName = leftFrontName;
        this.rightFrontName = rightFrontName;
        this.FourWheelDrive = false;
    }

    public TankDrive(Telemetry telemetry,
                     boolean useBrakeMode,
                     @NotNull String leftFrontName,
                     @NotNull String rightFrontName,
                     @NotNull String leftRearName,
                     @NotNull String rightRearName) { //4wd tank
        super(telemetry, useBrakeMode);
        this.leftFrontName = leftFrontName;
        this.rightFrontName = rightFrontName;
        this.leftRearName = leftRearName;
        this.rightRearName = rightRearName;
        this.FourWheelDrive = true;
    }

    @Override
    public void initDriveTrain(HardwareMap hardwareMap) {
        leftFront = (DcMotorEx)hardwareMap.get(DcMotor.class, leftFrontName);
        rightFront = (DcMotorEx)hardwareMap.get(DcMotor.class, rightFrontName);
        leftRear = (DcMotorEx)hardwareMap.get(DcMotor.class, leftRearName);
        rightRear = (DcMotorEx)hardwareMap.get(DcMotor.class, rightRearName);
    }

    /**
     * @param args, an array of doubles that correspond to [1] == y, [2] == turn
     */
    @Override
    public void drive(double ...args) {
        double x = args[0]; // Ignore this argument
        double y = args[1];
        double turn = args[2];

        double left = y + turn;
        double right = y - turn;

        left = Range.clip(left, -1, 1);
        right = Range.clip(right, -1, 1);

        if (!FourWheelDrive) {
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
    public void turn(double power) {
        drive(0.0,power);
    }
}
