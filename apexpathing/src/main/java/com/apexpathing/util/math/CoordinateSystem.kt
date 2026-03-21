package com.apexpathing.util.math

import kotlin.math.PI

interface CoordinateSystem {
    fun toApexCoordinates(pose: Pose): Pose
    fun fromApexCoordinates(pose: Pose): Pose
}

object ApexCoordinates : CoordinateSystem {
    override fun toApexCoordinates(pose: Pose): Pose = pose
    override fun fromApexCoordinates(pose: Pose): Pose = pose
}

object PedroCoordinates : CoordinateSystem {
    override fun toApexCoordinates(pose: Pose): Pose =
        pose.rotated(PI / 2) + Pose(72.0, 72.0)
    override fun fromApexCoordinates(pose: Pose): Pose =
        (pose - Pose(72.0, 72.0)).rotated(-PI / 2)
}
