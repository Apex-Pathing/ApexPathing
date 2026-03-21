package com.xpathing.main.follower.paths
data class Pose(
    var x : Double,
    var y : Double,
    var heading : Double
) {
//    fun addPose(other: Pose): Pose {
//        return Pose(
//            x + other.x,
//            y + other.y,
//            Math.toRadians(normalize(heading + other.heading))
//        )
//    }
    operator fun plus(other : Pose) : Pose {
        return Pose(
            x + other.x,
            y + other.y,
            Math.toRadians(normalize(heading + other.heading))
        )
    }
    operator fun minus(other : Pose) : Pose {
        return Pose(
            x - other.x,
            y - other.y,
            Math.toRadians(normalize(heading - other.heading))
        )
    }
    fun rotate(angle : Double) : Pose {
        val cosA = Math.cos(angle)
        val sinA = Math.sin(angle)

        return Pose(
            x * cosA - y * sinA,
            x * sinA + y * cosA,
            normalize(heading + angle)
        )
    }
//    fun subtract(other: Pose): Pose {
//        return Pose(
//            x - other.x,
//            y - other.y,
//            Pose.normalize(heading - other.heading)
//        )
//    }
    fun getDistance(other : Pose) : Double {
        val dX = other.x - x
        val dY = other.y - y
        return Math.hypot(dX, dY)
    }
    operator fun times(k: Double): Pose {
        return Pose(x * k, y * k, heading * k)
    }
    fun copyPose(): Pose {
        return Pose(x, y, heading)
    }


    companion object {
        fun normalize(angle: Double): Double {
            var a = angle
            while (a > Math.PI) {
                a -= 2 * Math.PI
            }
            while (a < -Math.PI) {
                a += 2 * Math.PI
            }
            return a
        }
    }

}
