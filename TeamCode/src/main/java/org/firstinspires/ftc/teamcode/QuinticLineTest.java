package org.firstinspires.ftc.teamcode;

import com.apexpathing.drivetrain.MecanumConstants;
import com.apexpathing.drivetrain.MecanumDrive;
import com.apexpathing.follower.QuinticHermiteSpline;
import com.apexpathing.follower.TimeTrajectory;
import com.apexpathing.localization.PinpointLocalizer;
import com.apexpathing.util.math.Pose;
import com.apexpathing.util.math.Vector;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="Quintic Test - Straight Line", group="Drive")
public class QuinticLineTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        // 1. Initialize Localizer
        PinpointLocalizer localizer = new PinpointLocalizer(hardwareMap, "pp", 0, 0);
        localizer.init();

        // 2. Initialize Drivetrain
        MecanumConstants constants = new MecanumConstants();
        MecanumDrive drive = new MecanumDrive(
                hardwareMap,
                constants
        );

        // 3. Define Trajectory
        double T = 1.8;
        // X-Path: Start 0, End 24. End velocity/accel 0.
        // Y-Path: Start 0, End 24. End velocity/accel 0.
        // Note: QuinticHermiteSpline is t from 0 to 1.
        // To simulate virtual tangents like original code, we can adjust end velocities.
        // In the original, xPath had start velocity 30.
        QuinticHermiteSpline spline = new QuinticHermiteSpline(
                new Vector(0, 0), new Vector(30, 0), new Vector(0, 0),
                new Vector(24, 24), new Vector(0, 30), new Vector(0, 0)
        );

        TimeTrajectory trajectory = new TimeTrajectory(
                new QuinticHermiteSpline[]{spline},
                new double[]{T},
                new double[]{0, Math.PI/4} // Start heading 0, end heading 45 deg
        );

        telemetry.addLine("The bot will move from (0,0) to (24, 24)");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        /*
        drive.followTrajectory(trajectory);

        while (opModeIsActive() && drive.isBusy()) {
            drive.update();

            Pose currentPose = localizer.getPose();
            telemetry.addData("X", currentPose.x());
            telemetry.addData("Y", currentPose.y());
            telemetry.addData("Heading", Math.toDegrees(currentPose.heading()));
            telemetry.update();
        }

        drive.setDrivePowers(new Pose(0, 0, 0));

         */
    }
}
