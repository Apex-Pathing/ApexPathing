package com.apexpathing.drivetrain.drivetrainTest;

import com.apexpathing.drivetrain.MecanumDrive;
import com.apexpathing.drivetrain.MecanumConstants;
import com.apexpathing.localization.PinpointLocalizer;
import com.apexpathing.util.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Beta test opmode for MecanumDrive :)
 * @author Sohum Arora - 22985 Paraducks
 */
@TeleOp(name = "MecanumDrive Test", group = "Apex beta test")
public class MecanumDriveTest extends LinearOpMode {
    private static final String lF  = "leftFront";
    private static final String rF = "rightFront";
    private static final String lR= "leftRear";
    private static final String rR = "rightRear";
    //todo change these names accordingly
    private boolean fieldCentric = false; //todo toggle y for fieldcentric/botcentric
    private boolean brakeMode = false;
    private boolean lastY = false;
    private boolean lastB = false;
    private boolean lastStart = false;
    PinpointLocalizer localizer;
    String localizerName = "localizer"; //todo change as required
    double localizerXOffset = 0.0;//todo replace with actual offset
    double localizerYOffset = 0.0; //todo replace with actual offset

    @Override
    public void runOpMode() {

        localizer = new PinpointLocalizer(hardwareMap, localizerName,localizerXOffset,localizerYOffset);
        localizer.init();
        MecanumConstants constants = new MecanumConstants();
        MecanumDrive drive = new MecanumDrive(
                hardwareMap, localizer, telemetry, constants,
                lF, rF, lR, rR
        );


        telemetry.addLine("Mecanum Drivetrain Movement Test");
        telemetry.addLine("Y - toggle field centric and bot centric");
        telemetry.addLine("B - toggle brake mode on and off");
        telemetry.update();

        waitForStart();
        drive.startTeleopDrive(brakeMode);

        while (opModeIsActive()) {
            localizer.update();
            Pose currentPose = localizer.getPose();

            double x = gamepad1.left_stick_x;
            double y = -gamepad1.left_stick_y;
            double turn = gamepad1.right_stick_x;

            if (gamepad1.y && !lastY) {
                fieldCentric = !fieldCentric;
                telemetry.addLine("Field-Centric: " + fieldCentric);
            }

            if (gamepad1.b && !lastB) {
                brakeMode = !brakeMode;
                drive.startTeleopDrive(brakeMode);
                telemetry.addLine("Brake Mode: " + brakeMode);
            }

            if (gamepad1.start && !lastStart) {
                drive.breakFollowing();
                telemetry.addLine("Motors stopped (breakFollowing)");
            }

            lastY= gamepad1.y;
            lastB = gamepad1.b;
            lastStart = gamepad1.start;

            if (!gamepad1.start) {
                if (fieldCentric) {
                    drive.fieldCentricDrive(x, y, turn, currentPose.heading());
                } else {
                    drive.botCentricDrive(x, y, turn);
                }
            }


            telemetry.addData("Bot pose ", currentPose);
            telemetry.addData("x ", currentPose.x());
            telemetry.addData("y ", currentPose.y());
            telemetry.addData("heading ", currentPose.heading());
            telemetry.addData("Brake mode ", brakeMode);

            if (fieldCentric) {
                telemetry.addLine("Field Centric drive");
            }
            else {
                telemetry.addLine("Bot centric drive");
            }
            telemetry.update();
        }
    }
}