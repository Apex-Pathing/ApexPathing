package com.apexpathing.follower

import com.apexpathing.drivetrain.Drivetrain
import com.apexpathing.util.Controller
import com.apexpathing.util.math.Pose
import kotlin.math.cos
import kotlin.math.sin

/**
 * Basic point-to-point follower class
 * @author Krish Joshi - 26192 Heatwaves
 * @author Sohum Arora 22985 Paraducks
 * @author Topher Fontana -- 23571 Alum
 */
class PTPFollower(
    private val xController: Controller,
    private val yController: Controller,
    private val headingController: Controller,
    private val paths: List<Path>,
) : Follower() {
    override var currentPath = paths[0]

    private var pathCounter: Int = 0

    private var target: Pose = Pose(0.0, 0.0, 0.0)

    override var isFinished: Boolean = false

    override fun setTarget(target: Pose) {
        this.target = target
        xController.setGoal(target.x)
        yController.setGoal(target.y)
        headingController.setGoal(target.heading)
        isFinished = false
    }

    override fun update(currentPose: Pose): DoubleArray {
        if (isFinished) return doubleArrayOf(0.0,0.0,0.0)

        if (currentPath.isFinished(currentPose)) {
            if (pathCounter < paths.size - 1) {
                currentPath = paths[++pathCounter]
                setTarget(currentPath.sample(1.0).pose)
            } else {
                isFinished = true
                return doubleArrayOf(0.0,0.0,0.0)
            }
        }

        val xPower    = xController.calculate(currentPose.x)
        val yPower    = yController.calculate(currentPose.y)
        val turnPower = headingController.calculate(currentPose.heading)

        val cos = cos(-currentPose.heading)
        val sin = sin(-currentPose.heading)
        val robotX = xPower * cos - yPower * sin
        val robotY = xPower * sin + yPower * cos

        return doubleArrayOf(robotX, robotY, turnPower)
    }
}