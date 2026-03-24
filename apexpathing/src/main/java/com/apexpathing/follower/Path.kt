package com.apexpathing.follower

import com.apexpathing.util.math.Pose

class Path(private val trajectories: List<Trajectory>, private val threshold: Double) : Trajectory {
    fun isFinished(currentPose: Pose): Boolean {
        if (trajectories.isEmpty()) return true
        return currentPose.nearPose(this.trajectories.last().sample(1.0).pose, threshold)
    }
    
    override fun sample(t: Double): TrajectorySample {
        if (trajectories.isEmpty()) {
            return TrajectorySample(Pose(), Pose(), Pose())
        }

        val clampedT = t.coerceIn(0.0, 1.0)
        val scaledT = clampedT * trajectories.size
        var index = scaledT.toInt()

        if (index >= trajectories.size) {
            index = trajectories.size - 1
        }

        var localT = scaledT - index
        if (index == trajectories.size - 1 && clampedT >= 1.0) {
            localT = 1.0
        }

        return trajectories[index].sample(localT)
    }

    override fun length(): Double {
        return trajectories.sumOf { it.length() }
    }
}
