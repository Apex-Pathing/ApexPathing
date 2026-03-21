package com.xpathing.util.math


import kotlin.math.PI
import kotlin.math.cos
import kotlin.math.sin
import kotlin.math.sqrt


data class Pose(
    @get:JvmName("x") var x: Double = 0.0,
    @get:JvmName("y") var y: Double = 0.0,
    @get:JvmName("heading") var heading: Double = 0.0,
    private var _coordSystem: CoordinateSystem
) {
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
        val otherPose = otherPose
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
    operator fun plus(vec: Vector) = this + vec.asPose()

    /**
     * Subtracts another pose from this pose
     *
     * @param otherPose the pose to be subtracted
     * @return the resulting difference as a pose
     */
    operator fun minus(otherPose: Pose): Pose {
        val otherPose = otherPose
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
    operator fun minus(vec: Vector) = this - vec.asPose()

    /**
     * Multiplies the x-y values by a certain amount
     *
     * @param scalar the scale factor to multiply by
     * @return the scaled pose
     */
    operator fun times(scalar: Double) = apply {
        x *= scalar
        y *= scalar
    }

    /**
     * Divides the x-y values by a certain amount
     *
     * @param scalar the scale factor to divide by
     * @return the scaled pose
     */
    operator fun div(scalar: Double) = apply {
        x /= scalar
        y /= scalar
    }

    /**
     * Negates the x-y values, and negates the heading as well
     *
     * @return negated form of this pose
     */
    operator fun unaryMinus() = apply {
        x *= -1
        y *= -1
        heading *= -1
    }
    fun copyPose(): Pose = Pose(x, y, heading, Vector.coordSys)

    /**
    *@param at A [Double] that we can flip the pose over (y = at)
    *@return A [Pose] that contains the reflection of the pose over the x-axis parallel line
    */
    fun reflectX(at: Double) = apply {
        y = 2 * at - y
        heading = normalize(-heading)
    }

    /**
    *@param at A [Double] that we can flip the pose over (x = at)
    *@return A [Pose] that contains the reflection of the pose over the y-axis parallel line
    */
    fun reflectY(at: Double) = apply {
        x = 2 * at - x
        heading = normalize(PI - heading)
    }

    /**
    *@param at A [Double] offset for the line y = x + at
    *@return A [Pose] that contains the reflection of the pose over the line y = x + at
    */
    fun reflectYX(at: Double): Pose {
        val oldX = x
        x = y - at
        y = oldX + at
        heading = normalize(PI / 2.0 - heading)
        return this
    }

    fun normalize(angle: Double): Double {
        var a = angle % (2 * PI)
        if (a > PI) a -= 2 * PI
        if (a <= -PI) a += 2 * PI
        return a

    }
    fun rotate(angle: Double): Pose {
        val cosA = cos(angle)
        val sinA = sin(angle)
        return Pose(
            x * cosA - y * sinA,
            x * sinA + y * cosA,
            normalize(heading + angle),
            _coordSystem = Vector.coordSys
        )
    }
    object coordSys : CoordinateSystem {
        override fun toApexCoordinates(pose: Pose): Pose {
            TODO("Not yet implemented")
        }

        override fun fromApexCoordinates(pose: Pose): Pose {
            TODO("Not yet implemented")
        }

    }
}

