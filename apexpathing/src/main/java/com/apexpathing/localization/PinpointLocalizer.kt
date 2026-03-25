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
 * TODO: Add StartingPose logic
 */
class PinpointLocalizer(
    private val hardwareMap: HardwareMap,
    private val deviceName: String,
    var xOffset: Double = 0.0,
    var yOffset: Double = 0.0
) : LocalizerBase() {

    private lateinit var pinpoint: GoBildaPinpointDriver

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

        lastPosition = currentPosition
        currentPosition = Pose(
            pos.getX(DistanceUnit.INCH),
            pos.getY(DistanceUnit.INCH),
            pinpoint.getHeading(AngleUnit.RADIANS)
        )

        currentVelocity.x = (currentPosition.x - lastPosition.x)
        currentVelocity.y = (currentPosition.y - lastPosition.y)
        currentVelocity.theta = (currentPosition.heading - lastPosition.heading)
    }

    override fun getPose(): Pose = currentPosition

    override fun getVelocity(): Pose = Pose(currentVelocity.x, currentVelocity.y, 0.0)

    override fun setPose(pose: Pose) {
        currentPosition = pose
        lastPosition = pose
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

    override fun initLocalizer(hardwareMap: HardwareMap) {
        init()
    }
}
