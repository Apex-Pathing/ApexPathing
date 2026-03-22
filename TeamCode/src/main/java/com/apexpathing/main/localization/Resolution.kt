package com.apexpathing.main.localization

import com.apexpathing.util.units.DistanceUnit

const val goBILDA_SWINGARM_POD = 13.26291192
const val goBILDA_4_BAR_POD = 19.89436789

public sealed class Resolution(open val value:Double) {
    object FOUR_BAR: Resolution(goBILDA_4_BAR_POD)

    object SWINGARM: Resolution(goBILDA_SWINGARM_POD)

    data class CUSTOM(override val value: Double, val distanceUnit: DistanceUnit = DistanceUnit.MM): Resolution(
        when(distanceUnit) {
            DistanceUnit.INCH -> value * 39.37
            DistanceUnit.MM -> value
            DistanceUnit.CM -> value * 10
            DistanceUnit.M -> value * 1000
        }
    )
}
