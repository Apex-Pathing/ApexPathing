package com.xpathing.util.controllers;
//1. The Hardware Map (The Physical Layer)
//Standardized naming for motors and encoders.
//
//Mecanum/Swerve: 4 MotorEx objects (FL, FR, BL, BR).
//
//Tank: 2 or 4 MotorEx objects.
//
//Localizer: A reference to your Localizer interface (Pinpoint, 3-wheel, etc.).
//
//2. The State Variables (The "Now" Layer)
//Variables that track what the robot is doing at this exact millisecond.
//
//Pose2d currentPose: Updated every loop from the Localizer.
//
//Pose2d targetVelocity: The velocity requested by the Spline Follower.
//
//List<Double> lastWheelPowers: Used for your Hardware Caching to prevent redundant writes.
//
//3. The Kinematics Method (The Math Layer)
//This is the only part that changes between classes. It translates a global ChassisSpeeds vector into specific motor commands.
//
//setDrivePowers(Pose2d powers):
//
//Mecanum: Applies the X, Y, and Turn vector math.
//
//Swerve: Calculates the 4 pod angles and 4 wheel speeds.
//
//Tank: Averages the X/Turn vectors for Left and Right sides.
//
//4. The Update Loop (The "Heartbeat")
//A mandatory update() method called once per loop in your OpMode.
//
//Java
//
//public void update() {
//    localizer.update(); // Where am I?
//    currentPose = localizer.getPose();
//
//    // If following a path, the follower calculates the next 'Target Velocity'
//    // then calls setDrivePowers() automatically.
//}
//5. The Constraint Overrides (The Limits)
//Each drivetrain has different physical limits (e.g., Tank turns slower than Swerve). These should be pulled from your DriveConstants.
//
//The "Mog" Factor: The Drivetrain Interface
//To make your library truly professional, define a CustomDrive abstract class that they all extend. This ensures that no matter which drive you use, the code looks like this:
//
//Java
//
//public abstract class CustomDrive {
//    protected Localizer localizer;
//    protected HolonomicController controller; // The Spline Follower
//
//    // Every drive type MUST implement these
//    public abstract void setDrivePowers(Pose2d drivePowers);
//    public abstract void update();
//
//    // Shared functionality: "Mogging" starts here
//    public void followTrajectory(Trajectory traj) {
//        controller.startFollowing(traj);
//    }
//}
public class TankDrivetrain {

}
