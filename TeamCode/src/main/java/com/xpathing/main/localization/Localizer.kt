package com.xpathing.main.localization

<<<<<<< HEAD
import com.xpathing.main.follower.paths.Pose
import com.xpathing.util.math.Vector
import java.lang.annotation.Inherited

=======
import com.xpathing.main.follower.Follower
import com.xpathing.util.math.Pose
import com.xpathing.util.math.Vector
import java.lang.annotation.ElementType
import java.lang.annotation.Inherited

/**
*@author Topher Fontana (@moosecoding)
*@date 3/19/26 
*/
>>>>>>> 76fcf68cfb766210971a5f09e054adc7cb309329
@MustBeDocumented
@Inherited
@Target(AnnotationTarget.CLASS)
@Retention(AnnotationRetention.RUNTIME)
annotation class Pathing

@Target(AnnotationTarget.CLASS)
@Retention(AnnotationRetention.RUNTIME)
@Inherited
@MustBeDocumented
annotation class Localizer

<<<<<<< HEAD
abstract class LocalizerBase {
    protected var lastPosition: Pose = Pose(0.0, 0.0, 0.0)
    protected var lastVelocity: Vector = Vector(0.0, 0.0)

    var currentPosition: Pose = Pose(0.0, 0.0, 0.0)
    var currentVelocity: Vector = Vector(0.0, 0.0)
    var currentAcceleration: Vector = Vector(0.0, 0.0)

    abstract fun initLocalizer(deviceName : String)
    abstract fun update()
    abstract fun setPose(pose: Pose)

    protected fun updateKinematics() {
        currentVelocity = Vector(
            currentPosition.x - lastPosition.x,
            currentPosition.y - lastPosition.y
        )
        currentAcceleration = currentVelocity.subtract(lastVelocity)
        lastVelocity = currentVelocity.copy()
        lastPosition = currentPosition.copyPose()
    }
}
=======
abstract class LocalizerBase() {
    // Add the @Pathing annotation stuff here

    // Stores the last position so that way we can calculate velocity
    private var lastPosition: Pose = Pose(0.0,0.0,0.0)
    // Stores the last velocity so that way we can calculate acceleration
    private var lastVelocity: Vector = Vector(0.0,0.0)
    var currentPosition: Pose = Pose(0.0,0.0,0.0)
    var currentVelocity: Vector = Vector(0.0,0.0)
    var currentAcceleration: Vector = Vector(0.0,0.0)

    /**
     * This initializes the localizer's hardware map or whatever else needed
     */
    abstract fun initLocalizer()


    /**
    * This is the update class called when your localizer is attached using the @Pathing annotation
    */
    abstract fun update()

    /**
    *@param A [Pose] to set the location of the localizer to
    */
    abstract fun setPose(pose: Pose)
} 
>>>>>>> 76fcf68cfb766210971a5f09e054adc7cb309329
