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

@Autonomous(name="Quintic Curve Test", group="Drive")
public class QuinticCurveTest extends LinearOpMode {

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
        double totalTime = 3.0;
        double midTime = totalTime / 2.0;

        // Points: Start(0,0) -> Mid(24, 0) -> End(24, 24)
        Vector pStart = new Vector(0, 0);
        Vector pMid = new Vector(24, 0);
        Vector pEnd = new Vector(24, 24);

        // Mid-point velocity/tangent
        Vector vMid = new Vector(20, 20);

        // Segment 1: Start to Mid
        QuinticHermiteSpline seg1 = new QuinticHermiteSpline(
                pStart, new Vector(0, 0), new Vector(0, 0),
                pMid, vMid, new Vector(0, 0)
        );

        // Segment 2: Mid to End
        QuinticHermiteSpline seg2 = new QuinticHermiteSpline(
                pMid, vMid, new Vector(0, 0),
                pEnd, new Vector(0, 0), new Vector(0, 0)
        );

        TimeTrajectory trajectory = new TimeTrajectory(
                new QuinticHermiteSpline[]{seg1, seg2},
                new double[]{midTime, midTime},
                new double[]{0, Math.PI/4, Math.PI/2} // 0 deg, 45 deg, 90 deg
        );

        telemetry.addLine("The bot will follow a curved path from (0,0) -> (24,0) -> (24,24)");
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
