package com.xpathing.util.math


import kotlin.math.hypot
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.sin

data class Vector(
    var x: Double,
    var y: Double
) {
    fun add(other: Vector) = Vector(x + other.x, y + other.y)
    fun subtract(other: Vector) = Vector(x - other.x, y - other.y)
    fun scale(k: Double) = Vector(x * k, y * k)
    fun magnitude() = hypot(x, y)
    fun normalize() = if (magnitude() == 0.0) Vector(0.0, 0.0) else scale(1.0 / magnitude())
    fun dot(other: Vector) = x * other.x + y * other.y
    fun angle() = atan2(y, x)
    fun copy() = Vector(x, y)

    companion object {
        fun fromPolar(magnitude: Double, angle: Double) =
            Vector(magnitude * cos(angle), magnitude * sin(angle))
    }
}