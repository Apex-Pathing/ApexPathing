package com.apexpathing.localization

import com.apexpathing.util.math.Pose
import com.apexpathing.util.math.Vector
import com.pedropathing.ftc.localization.RevHubIMU
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.util.ElapsedTime
import kotlin.math.cos
import kotlin.math.sin

/**
 * @author Topher Fontana on 3-21-26
 *
 * @param wheelRadius a [Double] that is the radius of the odometry pods, assume a standard size unless user wants seperate
 * @param hubOrientation a [RevHubOrientationOnRobot] that gives us the orientation of the hub on the robot
 * @param leftPodName a [String] representing the name of an odometry pod
 * @param rightPodName a [String] representing the name of an odometry pod
 * @param leftPodResolution a [Resolution] representing the resolution of an odometry pod
 * @param rightPodResolution a [Resolution] representing the resolution of an odometry pod
 */
class TwoWheelLocalizer(val wheelRadius:Double = 48.0, val hubOrientation: RevHubOrientationOnRobot, private var leftPodName: String, private var rightPodName: String, private var leftPodResolution: Resolution, private var rightPodResolution: Resolution, private var hubName:String = "imu"): LocalizerBase() {
    lateinit var odometryPods: Array<OdometryPod>

    // Temporarily zero while i fix this
    private var width: Double = 0.0

    private var lastTimeStamp: Double = 0.0

    private var strafeOffset: Double = 0.0

    private val timer: ElapsedTime = ElapsedTime()

    private val imu: RevHubIMU = RevHubIMU()

    override fun initLocalizer(hardwareMap: HardwareMap) {
        imu.initialize(hardwareMap, hubName, hubOrientation)
        odometryPods[0] = OdometryPod(hardwareMap.get(DcMotor::class.java, leftPodName) as DcMotorEx, leftPodResolution)
        odometryPods[1] = OdometryPod(hardwareMap.get(DcMotor::class.java, rightPodName) as DcMotorEx, rightPodResolution)
    }

    override fun update() {
        val timestamp: Double = timer.milliseconds()
        val dt = timestamp - lastTimeStamp
        lastTimeStamp = timestamp

        val dLW = odometryPods[0].getAngleTravelled() * wheelRadius
        val dRW = odometryPods[1].getAngleTravelled() * wheelRadius

        val dForward = (dLW + dRW) / 2
        val thetaMid = currentPosition.heading/2 + imu.heading/2

        currentPosition = Pose(currentPosition.x + dForward * cos(thetaMid), currentPosition.y + dStrafe * sin(thetaMid), currentPosition.heading + dHeading)

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
    }

    fun resetYaw() {
        imu.resetYaw()
    }
}