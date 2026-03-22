package com.apexpathing.localization

import com.apexpathing.util.math.Pose
import com.apexpathing.geometry.Vector
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.HardwareMap
import kotlin.math.cos
import kotlin.math.sin
import com.qualcomm.robotcore.util.ElapsedTime

/**
 * @author Topher Fontana on 3-21-26
 *
 * @param wheelRadius a [Double] that is the radius of the odometry pods, assume a standard size unless user wants seperate
 * @param leftPodName a [String] representing the name of an odometry pod
 * @param rightPodName a [String] representing the name of an odometry pod
 * @param strafePodName a [String] representing the name of an odometry pod
 * @param leftPodResolution a [Resolution] representing the resolution of an odometry pod
 * @param rightPodResolution a [Resolution] representing the resolution of an odometry pod
 * @param strafePodResolution a [Resolution] representing the resolution of an odometry pod
 */
class ThreeWheelLocalizer(val wheelRadius:Double = 48.0, private var leftPodName: String, private var rightPodName: String, private var strafePodName: String, private var leftPodResolution: Resolution, private var rightPodResolution: Resolution, private var strafePodResolution: Resolution): LocalizerBase() {
    private lateinit var odometryPods: Array<OdometryPod>

    // Temporarily zero while i fix this
    private var width: Double = 0.0

    private var lastTimeStamp: Double = 0.0

    private var strafeOffset: Double = 0.0

    private val timer: ElapsedTime = ElapsedTime()

    override fun initLocalizer(hardwareMap: HardwareMap) {
        odometryPods[0] = OdometryPod( hardwareMap.get(DcMotor::class.java, leftPodName) as DcMotorEx, leftPodResolution)
        odometryPods[1] = OdometryPod( hardwareMap.get(DcMotor::class.java, rightPodName) as DcMotorEx, rightPodResolution)
        odometryPods[2] = OdometryPod( hardwareMap.get(DcMotor::class.java, strafePodName) as DcMotorEx, strafePodResolution)
    }

    override fun update() {
        val timestamp: Double = timer.milliseconds()
        val dt = timestamp - lastTimeStamp
        lastTimeStamp = timestamp

        val dLW = odometryPods[0].getAngleTravelled() * wheelRadius
        val dRW = odometryPods[1].getAngleTravelled() * wheelRadius
        val dSW = odometryPods[2].getAngleTravelled() * wheelRadius

        val dHeading = (dLW - dRW) / width
        val dForward = (dLW + dRW) / 2
        val dStrafe = dSW - strafeOffset * dHeading

        val thetaMid = currentPosition.heading + dHeading/2

        // Fix the coordinate system issue, rn this is in mm
        lastPosition =  currentPosition
        currentPosition = Pose(currentPosition.x + dForward * cos(thetaMid), currentPosition.y + dStrafe * sin(thetaMid), currentPosition.heading + dHeading)

        lastVelocity = currentVelocity
        currentVelocity = Vector((currentPosition.x - lastPosition.x) / dt, (currentPosition.y - lastPosition.y) / dt)

        currentAcceleration = currentVelocity.subtract(lastVelocity).scale(1/dt)

        for(o in odometryPods) {
            o.update()
        }
    }

    override fun setPose(pose: Pose) {
        currentPosition = pose
        lastPosition = pose
    }
}
