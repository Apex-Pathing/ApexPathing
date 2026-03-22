package com.apexpathing.util.math

interface CoordinateSystem {
    fun toApexCoordinates(pose: Pose): Pose
    fun fromApexCoordinates(pose: Pose): Pose
}