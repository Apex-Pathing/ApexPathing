package com.apexpathing.main.localization

import com.apexpathing.util.units.DistanceUnit
import com.qualcomm.robotcore.hardware.DcMotorEx

class OdometryPod(val encoder: DcMotorEx, val resolution: Resolution) {
    private var lastPosition = 0
    private var currentPosition: Int = 0

    fun getDistance():Int {
        return currentPosition - lastPosition
    }

    fun getAngleTravelled(): Double {
        return getDistance() / resolution.value
    }

    fun update() {
        currentPosition = encoder.currentPosition
        lastPosition = currentPosition
    }
}