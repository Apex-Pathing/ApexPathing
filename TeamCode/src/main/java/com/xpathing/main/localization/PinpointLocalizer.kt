
package com.xpathing.main.localization

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver
import com.qualcomm.robotcore.hardware.HardwareMap

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D
import com.xpathing.util.math.Pose;
import com.xpathing.util.math.Vector

@Localizer
object PinpointLocalizer : LocalizerBase() {

    private lateinit var pinpoint: GoBildaPinpointDriver

    lateinit var hardwareMap: HardwareMap

    var xOffset: Double = 0.0
    var yOffset: Double = 0.0

    override fun initLocalizer(deviceName : String) {

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

        lastPosition = currentPosition.copyPose()

        currentPosition = Pose(
            pos.getX(DistanceUnit.INCH),
            pos.getY(DistanceUnit.INCH),
            pinpoint.getHeading(AngleUnit.RADIANS),
            _coordSystem = Vector.coordSys
        )

        updateKinematics()
    }

    override fun setPose(pose: Pose) {
        currentPosition = pose.copyPose()
        lastPosition = pose.copyPose()
        pinpoint.setPosition(
            Pose2D(
                DistanceUnit.INCH,
                pose.x,
                pose.y,
                AngleUnit.RADIANS,
                Math.toRadians(pose.heading)
            )
        )
    }
}
