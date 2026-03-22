@file:Suppress("unused")

package com.apexpathing.util.math

import com.apexpathing.geometry.Vector
import kotlin.math.PI
import kotlin.math.cos
import kotlin.math.sin
import kotlin.math.sqrt

/**
 * Class to represent a position in 2d space consisting of x-y coordinates and a heading orientation
 *
 * Provides methods for arithmetic functions with positions, flexible coordinate systems
 * Methods include:
 *   Utility Functions:
 *    - distanceTo
 *    - distanceFrom
 *    - distanceSquaredFrom
 *    - asVector
 *    - inCoordinateSystem
 *
 *   Transformation functions:
 *    - reflectX
 *    - withReflectedX
 *    - reflectY
 *    - withReflectedY
 *    - rotate
 *    - rotated
 *
 * @author Achintya Akula - 30099 OmicronX
 * @author Sohum Arora - 22985 Paraducks
 * @author Topher Fontana - 23571 jEdison Knights
 */
data class Pose
    @JvmOverloads constructor(
        @get:JvmName("x") var x: Double = 0.0,
        @get:JvmName("y") var y: Double = 0.0,
        @get:JvmName("heading") var heading: Double = 0.0,
        private var _coordSystem: CoordinateSystem = ApexCoordinates
    )
{
    @get:JvmName("coordinateSystem")
    val coordSystem
        get() = _coordSystem

    /**
     * Calculates the distance to another pose
     * Exactly the same as distanceFrom() however is included for user flexibility
     *
     * @return the distance to another pose
     */
    infix fun distanceTo(otherPose: Pose): Double = sqrt(distanceSquaredFrom(otherPose))

    /**
     * Calculates the distance from another pose to this pose
     * Exactly the same as distanceTo() however is included for user flexibility
     *
     * @return the distance from another pose
     */
    infix fun distanceFrom(otherPose: Pose): Double = distanceTo(otherPose)

    /**
     * Calculates the squared distance between one pose and another pose
     *
     * @return the squared distance to another pose
     */
    fun distanceSquaredFrom(otherPose: Pose): Double {
        val deltaX = otherPose.x - this.x
        val deltaY = otherPose.y - this.y
        return deltaX * deltaX + deltaY * deltaY
    }

    /**
     * Converts the pose to a vector
     *
     * @return the vector form of the current pose
     */
    fun asVector(): Vector = Vector(x, y)

    /**
     * Converts this pose to a specified coordinate system
     *
     * @param coordSys the coordinate system to convert to
     * @return the pose converted to the specified coordinate system
     */
    fun inCoordinateSystem(coordSys: CoordinateSystem) =
        coordSys.fromApexCoordinates(this.coordSystem.toApexCoordinates(this))

    /**
     * Adds this pose and another pose together
     * Converts all to the same coordinate system
     *
     * @param otherPose the pose to be added
     * @return the resulting sum as a pose
     */
    operator fun plus(otherPose: Pose): Pose {
        val otherPose = otherPose.inCoordinateSystem(coordSystem)
        return Pose(
            x + otherPose.x,
            y + otherPose.y,
            heading + otherPose.heading,
            coordSystem
        )
    }

    /**
     * Adds a vector to this pose
     *
     * @param vec the vector to be added
     * @return the resulting sum as a pose
     */
    operator fun plus(vec: Vector) =
        this + vec.asPose()

    /**
     * Subtracts another pose from this pose
     *
     * @param otherPose the pose to be subtracted
     * @return the resulting difference as a pose
     */
    operator fun minus(otherPose: Pose): Pose {
        val otherPose = otherPose.inCoordinateSystem(coordSystem)
        return Pose(
            x - otherPose.x,
            y - otherPose.y,
            heading - otherPose.heading,
            coordSystem
        )
    }

    /**
     * Subtracts a vector from this pose
     *
     * @param vec the vector to be subtracted
     * @return the resulting difference as a pose
     */
    operator fun minus(vec: Vector) =
        this - vec.asPose()

    /**
     * Multiplies the x-y values by a certain amount
     *
     * @param scalar the scale factor to multiply by
     * @return the scaled pose
     */
    operator fun times(scalar: Double) =
        Pose(
            x * scalar,
            y * scalar,
            heading,
            coordSystem
        )
    /**
     * Divides the x-y values by a certain amount
     *
     * @param scalar the scale factor to divide by
     * @return the scaled pose
     */
    operator fun div(scalar: Double) =
        Pose(
            x / scalar,
            y / scalar,
            heading,
            coordSystem
        )

    /**
     * Negates the x-y values, and negates the heading as well
     *
     * @return negated form of this pose
     */
    operator fun unaryMinus() =
        Pose(
            x * -1,
            y * -1,
            heading * -1,
            coordSystem
        )

    /**
     * Function to reflect a pose over (y = at)
     * Applies reflection to current pose
     *
     * @param at A [Double] that we can flip the pose over (y = at)
     * @return The reflected [Pose] that has been reflected over a x-axis parallel line
     */
    @JvmOverloads()
    fun reflectX(at: Double = 0.0) =
        apply {
            y = 2 * at - y
            heading = normalize(-heading)
        }

    /**
     * Function to reflect a pose over (y = at)
     * Creates a new pose with reflected coordinates
     *
     * @param at A [Double] that we can flip the pose over (y = at)
     * @return A [Pose] that is the reflection of this pose over a x-axis parallel line
     */
    @JvmOverloads()
    fun withReflectedX(at: Double = 0.0) =
        Pose(
            x,
            2 * at - y,
            normalize(-heading),
            coordSystem
        )

    /**
     * Function to reflect a pose over (x = at)
     * Applies reflection to current pose
     *
     * @param at A [Double] that we can flip the pose over (x = at)
     * @return The reflected [Pose] that has been reflected over a y-axis parallel line
     */
    @JvmOverloads()
    fun reflectY(at: Double = 0.0) =
        apply {
            x = 2 * at - x
            heading = normalize(PI - heading)
        }

    /**
     * Function to reflect a pose over (x = at)
     * Creates a new pose with reflected coordinates
     *
     * @param at A [Double] that we can flip the pose over (x = at)
     * @return A [Pose] that is the reflection of this pose over the y-axis parallel line
     */
    @JvmOverloads()
    fun withReflectedY(at: Double = 0.0) =
        Pose(
            2 * at - x,
            y,
            normalize(PI - heading),
            coordSystem
        )

    /**
     * Function to rotate a pose around the origin by an angle theta
     * Applies the rotation to the original pose
     *
     * @param theta The angle to rotate the position by
     * @param headingTheta The amount to rotate heading by, equals theta if not specified
     * @return The pose after rotation has been applied to it
     */
    @JvmOverloads()
    fun rotate(theta: Double, headingTheta: Double = theta) =
        apply {
            x = x * cos(theta) - y * sin(theta)
            y = x * sin(theta) + y * cos(theta)
            heading = heading + headingTheta
        }

    /**
     * Function to rotate a pose around the origin by an angle theta
     * Returns a new pose that is the rotated form of the original pose
     *
     * @param theta The angle to rotate the position by
     * @param headingTheta The amount to rotate heading by, equals theta if not specified
     * @return A new pose that is the rotated form of the current one
     */
    @JvmOverloads()
    fun rotated(theta: Double, headingTheta: Double = theta) =
        Pose(
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

    /**
     * Function to convert the position to a string for easy viewing
     *
     * @return String format of the pose
     */
    override fun toString(): String = "($x, $y, $heading)"


    /**
     * Debug Function to convert the position into a string that the user can use for debugging
     *
     * @return The string with x-y, heading, and coordSys information
     */
    fun debug(): String = "Pose { x: $x, y: $y, heading: $heading } in ${coordSystem.name()}"
}

/**
 * NOTE TO DEVS:
 *
 * At some point, extract the normalize function to a separate object that contains a bunch of Math Functions
 */
