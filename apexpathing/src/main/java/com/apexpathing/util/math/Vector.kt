@file:Suppress("unused")

package com.apexpathing.util.math

import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.hypot
import kotlin.math.sin

/**
 * Class to represent a 2d directional vector consisting of formats for both x-y and r-theta values
 *
 * Provides methods for arithmetic using vectors as well as for transformations
 * Methods include:
 *   - xComponent
 *   - yComponent
 *   - dotProduct
 *   - crossProduct
 *   - rotateVec
 *   - rotatedVec
 *   - asPose
 *
 * @author Achintya Akula - 30099 OmicronX
 * @author Sohum Arora
 */
data class Vector
    @JvmOverloads constructor(
        @get:JvmName("x") var x: Double = 0.0,
        @get:JvmName("y") var y: Double = 0.0
    )
{
    companion object {
        @JvmOverloads()
        fun fromPolar(r: Double = 0.0, theta: Double = 0.0) =
            Vector(r * cos(theta), r * sin(theta))
    }

    /**
     * The magnitude of the vector in polar form
     */
    @get:JvmName("magnitude")
    var magnitude: Double
        get() = hypot(x, y)
        set(value) {
            val ang = theta
            x = value * cos(ang)
            y = value * sin(ang)
        }

    /**
     * The angle of the vector in polar form
     */
    @get:JvmName("theta")
    var theta: Double
        get() = atan2(y, x)
        set(value) {
            val r = magnitude
            x = r * cos(value)
            y = r * sin(value)
        }

    /**
     * Function to get the x value of the given vector
     *
     * @return The x-value of the vector
     */
    fun xComponent() = x

    /**
     * Function to get the x value of the given vector
     *
     * @return The y-value of the vector
     */
    fun yComponent() = y

    /**
     * Calculates the dot product of this vector and another vector
     *
     * @param otherVec The Vector to dot-multiply by
     * @return the dot product of the two vectors
     */
    fun dotProduct(otherVec: Vector) =
        this * otherVec

    /**
     * Calculates the cross product of this vector and another vector
     *
     * @param otherVec the Vector to cross-multiply by
     * @return the cross product of the two vectors
     */
    fun crossProduct(otherVec: Vector) =
        ((x * otherVec.y) - (y * otherVec.x))

    /**
     * Function to rotate the given vector
     * Returns a new vector that is the rotated form of this vector
     *
     * @param ang The angle to rotate the vector by
     * @return A new vector that is the rotated form of this vector
     */
    fun rotateVec(ang: Double) =
        fromPolar(magnitude, normalize(this.theta + ang))

    /**
     * Function to rotate the given vector
     * Applies the rotation to this vector
     *
     * @param ang The angle to rotate the vector by
     * @return This vector with the rotation applied to it
     */
    fun rotatedVec(ang: Double) = apply {
       theta = normalize(theta + ang)
    }

    /**
     * Returns the Vector as a pose
     *
     * @return The Pose form of this vector
     */
    fun asPose() =
        Pose(x, y)

    /**
     * Function to add two vectors
     *
     * @param otherVec the vector to add to this vector
     * @return the sum of both vectors as a vector
     */
    operator fun plus(otherVec: Vector) =
        Vector(x + otherVec.x, y + otherVec.y)

    /**
     * Function to add a vector and a pose
     *
     * @param otherPose the pose to add to this vector
     * @return the sum as a vector
     */
    operator fun plus(otherPose: Pose) =
        Vector(x + otherPose.x, y + otherPose.y)

    /**
     * Function to subtract two vectors
     *
     * @param otherVec the vector to subtract from this vector
     * @return the difference of the two vectors as a vector
     */
    operator fun minus(otherVec: Vector) =
        Vector(x - otherVec.x, y - otherVec.y)

    /**
     * Function to subtract a pose from this vector
     *
     * @param otherPose the pose to subtract from this vector
     * @return the difference as a vector
     */
    operator fun minus(otherPose: Pose) =
        Vector(x - otherPose.x, y - otherPose.y)

    /**
     * Function to multiply two vectors together via dot product
     *
     * @param otherVec the vector to multiply with this vector
     * @return the product as a vector
     */
    operator fun times(otherVec: Vector) =
        ((x * otherVec.x) + (y * otherVec.y))

    /**
     * Function to multiply the vector by a certain scalar
     *
     * @param scalar the amount to multiply the vector by
     * @return The scaled form of the vector
     */
    operator fun times(scalar: Double) =
        Vector(x * scalar, y * scalar)

    /**
     * Function to divide the vector by a certain scalar
     * @param scalar the amount to divide the vector by
     * @return The quotient after scaling the vector
     */
    operator fun div(scalar: Double) =
        Vector(x / scalar, y / scalar)

    /**
     * Function to negate the vector
     *
     * @return the negated form of the vector
     */
    operator fun unaryMinus() = (this * -1.0)

    /**
     * Function to convert the vector to a string for easy viewing
     *
     * @return The string format of the current vector
     */
    override fun toString(): String = "<$x, $y>"

    /**
     * Debug function to convert the vector into a string that can be used for debugging
     * Provides more info than [toString]
     *
     * @return The vector with info about x-y values and r-theta values
     */
    fun debug(): String = "Vector <x: $x, y: $y>, <magnitude: $magnitude, θ: $theta>"

    /**
     * Function to get a copy of this vector
     *
     * @return a copy of this vector
     */
    fun copy(): Vector =
        Vector(x, y)
}