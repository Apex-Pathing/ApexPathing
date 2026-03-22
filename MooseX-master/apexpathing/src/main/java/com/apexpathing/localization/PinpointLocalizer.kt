package com.apexpathing.localization

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver
import com.qualcomm.robotcore.hardware.HardwareMap

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D

import com.apexpathing.util.math.Pose

/**
 * GoBilda Pinpoint odometry localizer
 * @Author Sohum Arora 22985
 */
class PinpointLocalizer(
    private val hardwareMap: HardwareMap,
    private val deviceName: String,
    var xOffset: Double = 0.0,
    var yOffset: Double = 0.0
) : Localizer {

    private lateinit var pinpoint: GoBildaPinpointDriver
    private var currentPose = Pose(0.0, 0.0, 0.0)
    private var lastPose = Pose(0.0, 0.0, 0.0)
    private var currentVelocity = Pose(0.0, 0.0, 0.0)

    fun init() {
        pinpoint = hardwareMap.get(GoBildaPinpointDriver::class.java, deviceName)
        pinpoint.setOffsets(xOffset, yOffset, DistanceUnit.INCH)
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
        pinpoint.setEncoderDirections(
            GoBildaPinpointDriver.EncoderDirection.FORWARD,
            GoBildaPinpointDriver.EncoderDirection.FORWARD
        )
        pinpoint.resetPosAndIMU()
    }

    override fun update() {
        pinpoint.update()
        val pos = pinpoint.position

        lastPose = currentPose
        currentPose = Pose(
            pos.getX(DistanceUnit.INCH),
            pos.getY(DistanceUnit.INCH),
            pinpoint.getHeading(AngleUnit.RADIANS)
        )

        currentVelocity = Pose(
            currentPose.x - lastPose.x,
            currentPose.y - lastPose.y,
            currentPose.heading - lastPose.heading
        )
    }

    override fun getPose(): Pose = currentPose

    override fun getVelocity(): Pose = currentVelocity

    override fun setPose(pose: Pose) {
        currentPose = pose
        lastPose = pose
        pinpoint.setPosition(
            Pose2D(
                DistanceUnit.INCH,
                pose.x,
                pose.y,
                AngleUnit.RADIANS,
                pose.heading
            )
        )
    }
}
