package com.xpathing.main.localization

import com.xpathing.main.follower.paths.Pose


object FieldCoordinates {
    const val fieldSize = 144.0
    const val halfField = fieldSize / 2

    // Field boundaries
    const val maxX = halfField
    const val minX = -halfField
    const val maxY = halfField
    const val minY = -halfField

    fun clamp(pose: Pose): Pose {
        return Pose(
            pose.x.coerceIn(minX, maxX),
            pose.y.coerceIn(minY, maxY),
            pose.heading
        )
    }

}