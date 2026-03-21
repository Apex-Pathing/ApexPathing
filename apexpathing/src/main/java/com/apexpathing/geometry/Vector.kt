package com.apexpathing.geometry

import com.apexpathing.util.math.Pose
import kotlin.math.hypot
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.sin

/**
 * Mutable 2D vector with polar coordinate support.
 * Used by MecanumDrive for drive power calculations.
 */
data class Vector(
    var x: Double,
    var y: Double
) {
    var magnitude: Double
        get() = hypot(x, y)
        set(value) {
            val n = normalize()
            x = n.x * value
            y = n.y * value
        }

    fun add(other: Vector) = Vector(x + other.x, y + other.y)
    fun subtract(other: Vector) = Vector(x - other.x, y - other.y)
    fun scale(k: Double) = Vector(x * k, y * k)
    fun normalize() = if (magnitude == 0.0) Vector(0.0, 0.0) else scale(1.0 / magnitude)
    fun dot(other: Vector) = x * other.x + y * other.y
    fun angle() = atan2(y, x)
    fun copy() = Vector(x, y)
    fun asPose() = Pose(x, y)

    fun rotateVector(angle: Double): Vector {
        val cosA = cos(angle)
        val sinA = sin(angle)
        val newX = x * cosA - y * sinA
        val newY = x * sinA + y * cosA
        x = newX
        y = newY
        return this
    }

    fun getTheta(): Double = atan2(y, x)
    fun getXComponent(): Double = x
    fun getYComponent(): Double = y

    operator fun plus(vec: Vector) = Vector(x + vec.x, y + vec.y)
    operator fun minus(vec: Vector) = Vector(x - vec.x, y - vec.y)
    operator fun times(vec: Vector) = Vector(x * vec.x, y * vec.y)
    operator fun times(scalar: Double) = Vector(x * scalar, y * scalar)

    companion object {
        fun fromPolar(magnitude: Double, angle: Double) =
            Vector(magnitude * cos(angle), magnitude * sin(angle))
    }
}
