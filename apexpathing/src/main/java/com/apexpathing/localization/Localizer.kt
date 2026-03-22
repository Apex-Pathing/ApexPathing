package com.apexpathing.localization


import com.apexpathing.geometry.Vector
import com.apexpathing.util.math.Pose
import com.qualcomm.robotcore.hardware.HardwareMap

/**
 *@author Topher Fontana (@moosecoding)
 *@date 3/19/26
 */



abstract class LocalizerBase() {
    // Add the @Pathing annotation stuff here

    // Stores the last position so that way we can calculate velocity
    var lastPosition: Pose = Pose(0.0,0.0,0.0, _coordSystem = Vector.coordSys)
    // Stores the last velocity so that way we can calculate acceleration
    var lastVelocity: Vector = Vector(0.0,0.0)
    var currentPosition: Pose = Pose(0.0,0.0,0.0, _coordSystem = Vector.coordSys)
    var currentVelocity: Vector = Vector(0.0,0.0)
    var currentAcceleration: Vector = Vector(0.0,0.0)



    /**
     * This initializes the localizer's hardware map or whatever else needed
     */
    abstract fun initLocalizer(hardwareMap: HardwareMap)


    /**
     * This is the update class called when your localizer is attached using the @Pathing annotation
     */
    abstract fun update()

    /**
     *@param A [Pose] to set the location of the localizer to
     */
    abstract fun setPose(pose: Pose)
}

