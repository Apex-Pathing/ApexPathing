package com.apexpathing.drivetrain;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.jetbrains.annotations.NotNull;

public class TankDrive extends Drivetrain {

    boolean FourWheelDrive;

    DcMotorEx leftFront, rightFront, leftRear, rightRear;
    String leftFrontName, rightFrontName, leftRearName, rightRearName;


    public TankDrive(HardwareMap hardwareMap, Telemetry telemetry,
                     boolean useBrakeMode,
                     @NotNull String leftFrontName,
                     @NotNull String rightFrontName
                     ) { //2wd tank drive constructor
        super(hardwareMap,telemetry, useBrakeMode);
        this.leftFrontName = leftFrontName;
        this.rightFrontName=rightFrontName;
        this.FourWheelDrive = false;
    }

    @Override
    public void initDriveTrain() {
        leftFront = (DcMotorEx)hardwareMap.get(DcMotor.class, leftFrontName);
        rightFront = (DcMotorEx)hardwareMap.get(DcMotor.class, rightFrontName);
    }

    public TankDrive(HardwareMap hardwareMap,
                     Telemetry telemetry,
                     boolean useBrakeMode,
                     @NotNull String leftFrontName,
                     @NotNull String rightFrontName,
                     @NotNull String leftRearName,
                     @NotNull String rightRearName) { //4wd tank
        super(hardwareMap, telemetry, useBrakeMode);
        leftFront = (DcMotorEx)hardwareMap.get(DcMotor.class, leftFrontName);
        rightFront = (DcMotorEx)hardwareMap.get(DcMotor.class, rightFrontName);
        leftRear = (DcMotorEx)hardwareMap.get(DcMotor.class, leftRearName);
        rightRear = (DcMotorEx)hardwareMap.get(DcMotor.class, rightRearName);

        this.FourWheelDrive = true;
    }

    /**
     * @param args, an array of doubles that correspond to [0] == y, [1] == turn
     */
    @Override
    public void drive(double ...args) {
        double y = args[0];
        double turn = args[1];

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
