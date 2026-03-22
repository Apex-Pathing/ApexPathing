package com.apexpathing.util.math

import kotlin.math.hypot
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.sin

data class Vector(
    var x: Double,
    var y: Double,
    var theta:Double = 0.0,
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
    fun asPose(heading: Double = 0.0): Pose = Pose(x, y, heading, _coordSystem = coordSys)

    object coordSys : CoordinateSystem {
        override fun toApexCoordinates(pose: Pose): Pose {
            TODO("Not yet implemented")
        }

        override fun fromApexCoordinates(pose: Pose): Pose {
            TODO("Not yet implemented")
        }
    }
    fun rotateVector(angle: Double): Vector {
        val cosA = cos(angle)
        val sinA = sin(angle)
        return Vector(
            x * cosA - y * sinA,
            x * sinA + y * cosA
        )
    }
    fun getAngleFromXY() : Double {
        return atan2(y,x)
    }
    fun getXComponent() : Double {return x}
    fun getYComponent() : Double {return y}

    operator fun plus(vec: Vector) = Vector(x + vec.x, y + vec.y)
    operator fun minus(vec: Vector) = Vector(x - vec.x, y - vec.y)
    operator fun times(vec : Vector) = Vector(x * vec.x, y * vec.y)
    operator fun times(scalar: Double) = Vector(x * scalar, y * scalar)
    companion object {
        fun fromPolar(magnitude: Double, angle: Double) =
            Vector(magnitude * cos(angle), magnitude * sin(angle))

    }
}