@file:Suppress("unused")

package com.apexpathing.util.math

import com.apexpathing.geometry.Vector
import kotlin.math.PI
import kotlin.math.cos
import kotlin.math.sin
import kotlin.math.sqrt

/**
 * @author Achintya Akula, Sohum Arora, Topher Fontana
 */
data class Pose(
    @get:JvmName("x") var x: Double = 0.0,
    @get:JvmName("y") var y: Double = 0.0,
    @get:JvmName("heading") var heading: Double = 0.0,
    private var _coordSystem: CoordinateSystem = ApexCoordinates
) {
    @get:JvmName("coordinateSystem")
    val coordSystem get() = _coordSystem

    infix fun distanceTo(otherPose: Pose): Double = sqrt(distanceSquaredFrom(otherPose))
    infix fun distanceFrom(otherPose: Pose): Double = distanceTo(otherPose)

    fun distanceSquaredFrom(otherPose: Pose): Double {
        val deltaX = otherPose.x - this.x
        val deltaY = otherPose.y - this.y
        return deltaX * deltaX + deltaY * deltaY
    }

    fun asVector(): Vector = Vector(x, y)

    fun inCoordinateSystem(coordSys: CoordinateSystem) =
        coordSys.fromApexCoordinates(this.coordSystem.toApexCoordinates(this))

    operator fun plus(otherPose: Pose): Pose {
        val converted = otherPose.inCoordinateSystem(coordSystem)
        return Pose(x + converted.x, y + converted.y, heading + converted.heading, coordSystem)
    }

    operator fun plus(vec: Vector) = this + vec.asPose()

    operator fun minus(otherPose: Pose): Pose {
        val converted = otherPose.inCoordinateSystem(coordSystem)
        return Pose(x - converted.x, y - converted.y, heading - converted.heading, coordSystem)
    }

    operator fun minus(vec: Vector) = this - vec.asPose()

    operator fun times(scalar: Double) = Pose(x * scalar, y * scalar, heading, coordSystem)
    operator fun div(scalar: Double) = Pose(x / scalar, y / scalar, heading, coordSystem)
    operator fun unaryMinus() = Pose(-x, -y, -heading, coordSystem)

    fun reflectX(at: Double) = apply {
        y = 2 * at - y
        heading = normalize(-heading)
    }

    fun withReflectedX(at: Double) = Pose(x, 2 * at - y, normalize(-heading), coordSystem)

    fun reflectY(at: Double) = apply {
        x = 2 * at - x
        heading = normalize(PI - heading)
    }

    fun withReflectedY(at: Double) = Pose(2 * at - x, y, normalize(PI - heading), coordSystem)

    fun rotate(theta: Double, headingTheta: Double = theta) = apply {
        val origX = x
        x = origX * cos(theta) - y * sin(theta)
        y = origX * sin(theta) + y * cos(theta)
        heading += headingTheta
    }

    fun rotated(theta: Double, headingTheta: Double = theta) = Pose(
        x * cos(theta) - y * sin(theta),
        x * sin(theta) + y * cos(theta),
        heading + headingTheta,
        coordSystem
    )

    fun normalize(angle: Double): Double {
        var a = angle % (2 * PI)
        if (a > PI) a -= 2 * PI
        if (a <= -PI) a += 2 * PI
        return a
    }
}
