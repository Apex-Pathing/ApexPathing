package com.apexpathing.drivetrain;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import com.apexpathing.util.math.Pose;
import com.apexpathing.util.math.Vector;
import com.apexpathing.hardware.MotorEx;
import com.apexpathing.hardware.AbsoluteAnalogEncoder;
import com.apexpathing.kinematics.ChassisSpeeds;
import com.apexpathing.kinematics.SwerveKinematics;
import com.apexpathing.kinematics.SwerveModuleState;
import com.apexpathing.localization.Localizer;
import com.apexpathing.follower.HolonomicTrajectoryFollower;

import java.util.ArrayList;
import java.util.List;

/**
 * SwerveDrive implementation following the 5-part structure.
  * @author Krish Joshi - 26192 Heatwaves
 */
public class SwerveDrive extends CustomDrive {
    private final SwerveModule[] modules = new SwerveModule[4];
    private final SwerveKinematics kinematics;
    private boolean locked = false;

    public SwerveDrive(HardwareMap hardwareMap, Localizer localizer, Vector[] moduleOffsets) {
        this.localizer = localizer;
        this.kinematics = new SwerveKinematics(moduleOffsets);
        this.controller = new HolonomicTrajectoryFollower();
        this.voltageSensor = hardwareMap.voltageSensor.iterator().next();

        String[] moduleNames = {"FL", "FR", "BL", "BR"};
        for (int i = 0; i < 4; i++) {
            MotorEx driveMotor = new MotorEx(hardwareMap.get(DcMotorEx.class, moduleNames[i] + "Drive"));
            Servo turnServo = hardwareMap.get(Servo.class, moduleNames[i] + "Turn");
            AnalogInput analogInput = hardwareMap.get(AnalogInput.class, moduleNames[i] + "Encoder");
            AbsoluteAnalogEncoder encoder = new AbsoluteAnalogEncoder(analogInput);

            modules[i] = new SwerveModule(driveMotor, turnServo, encoder);
        }
    }

    private Pose targetVelocity = new Pose(0, 0, 0);

    @Override
    public void setDrivePowers(Pose drivePowers) {
        this.targetVelocity = drivePowers;

        if (locked) {
            double[] lockAngles = {Math.PI / 4.0, -Math.PI / 4.0, 3.0 * Math.PI / 4.0, -3.0 * Math.PI / 4.0};
            for (int i = 0; i < 4; i++) {
                modules[i].setDesiredState(0, lockAngles[i], true);
            }
            return;
        }

        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(drivePowers.x(), drivePowers.y(), drivePowers.heading());
        SwerveModuleState[] states = kinematics.calculate(chassisSpeeds);

        for (int i = 0; i < 4; i++) {
            modules[i].setDesiredState(states[i].speed, states[i].angle);
        }
    }

    public void setLocked(boolean locked) {
        this.locked = locked;
    }

    public boolean isLocked() {
        return locked;
    }

    public List<Double> getModulePositions() {
        List<Double> positions = new ArrayList<>();
        for (SwerveModule module : modules) {
            positions.add(module.getWheelPosition());
        }
        return positions;
    }

    public List<Double> getModuleAngles() {
        List<Double> angles = new ArrayList<>();
        for (SwerveModule module : modules) {
            angles.add(module.getModuleAngle());
        }
        return angles;
    }

    public double[] getTargetAngles() {
        double[] targets = new double[4];
        for (int i = 0; i < 4; i++) {
            targets[i] = modules[i].getLastPosition() * 2.0 * Math.PI;
        }
        return targets;
    }

    public double[] getActualAngles() {
        double[] actuals = new double[4];
        for (int i = 0; i < 4; i++) {
            actuals[i] = modules[i].getModuleAngle();
        }
        return actuals;
    }

    public SwerveModule[] getModules() {
        return modules;
    }

    public Pose getTargetVelocity() {
        return targetVelocity;
    }
}
