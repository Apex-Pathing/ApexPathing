package com.xpathing.drivetrains;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.movement.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.movement.geometry.Vector2d;
import org.firstinspires.ftc.teamcode.movement.hardware.MotorEx;
import org.firstinspires.ftc.teamcode.movement.kinematics.ChassisSpeeds;
import org.firstinspires.ftc.teamcode.movement.kinematics.SwerveKinematics;
import org.firstinspires.ftc.teamcode.movement.kinematics.SwerveModuleState;
import org.firstinspires.ftc.teamcode.movement.localization.Localizer;
import com.xpathing.main.follower.xpathing.HolonomicTrajectoryFollower;

import java.util.ArrayList;
import java.util.List;

/**
 * SwerveDrive implementation following the 5-part structure.
 */
public class SwerveDrive extends CustomDrive {
    // 1. The Hardware Map (The Physical Layer)
    private final SwerveModule[] modules = new SwerveModule[4];
    private final SwerveKinematics kinematics;

    public SwerveDrive(HardwareMap hardwareMap, Localizer localizer, Vector2d[] moduleOffsets) {
        this.localizer = localizer;
        this.kinematics = new SwerveKinematics(moduleOffsets);
        this.controller = new HolonomicTrajectoryFollower(kinematics);

        // Standardized naming for motors, servos, and absolute encoders
        String[] moduleNames = {"FL", "FR", "BL", "BR"};
        for (int i = 0; i < 4; i++) {
            MotorEx driveMotor = new MotorEx(hardwareMap.get(DcMotorEx.class, moduleNames[i] + "Drive"));
            Servo turnServo = hardwareMap.get(Servo.class, moduleNames[i] + "Turn");
            AnalogInput absoluteEncoder = hardwareMap.get(AnalogInput.class, moduleNames[i] + "Encoder");

            modules[i] = new SwerveModule(driveMotor, turnServo, absoluteEncoder);
        }
    }

    // 2. The State Variables (The "Now" Layer)
    private Pose2d currentPose = new Pose2d(0, 0, 0);
    private Pose2d targetVelocity = new Pose2d(0, 0, 0);

    // 3. The Kinematics Method (The Math Layer)
    @Override
    public void setDrivePowers(Pose2d drivePowers) {
        this.targetVelocity = drivePowers;
        // Translates global ChassisSpeeds vector into specific motor commands
        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(drivePowers.x, drivePowers.y, drivePowers.heading);
        SwerveModuleState[] states = kinematics.calculate(chassisSpeeds);

        // Command the 4 modules to match those states
        for (int i = 0; i < 4; i++) {
            modules[i].setDesiredState(states[i].speed, states[i].angle);
        }
    }

    // 4. The Update Loop (The "Heartbeat")
    @Override
    public void update() {
        // Localizer update: Where am I?
        localizer.update();
        currentPose = localizer.getPose();

        // Update all 4 SwerveModule objects
        for (SwerveModule module : modules) {
            module.update();
        }

        // If following a path, the follower logic would be here
        // The follower calculates the next 'Target Velocity' then calls setDrivePowers()
    }

    // 5. Odometry Passthrough (The Limits)
    /**
     * Get current wheel positions and angles from the modules to feed back into the Localizer.
     * @return List of module positions (ticks/distance) and angles (radians).
     */
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

    /**
     * Telemetry hooks for FTC panels to debug module target vs actual angles.
     */
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

    public Pose2d getCurrentPose() {
        return currentPose;
    }

    public Pose2d getTargetVelocity() {
        return targetVelocity;
    }
}
