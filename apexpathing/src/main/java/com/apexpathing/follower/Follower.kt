package com.apexpathing.follower

import com.apexpathing.util.math.Pose

abstract class Follower {
    abstract var currentPath: Path
    private var currentT = 0.0

    open var isFinished = false

    abstract fun setTarget(target: Pose)

    /**
     * @return [DoubleArray] containing the powers to set the drivetrain to
     */
    abstract fun update(currentPose: Pose): DoubleArray
}