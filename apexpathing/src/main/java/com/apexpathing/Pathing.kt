package com.apexpathing

import android.content.Context
import com.apexpathing.drivetrain.Drivetrain
import com.apexpathing.follower.Follower
import com.apexpathing.localization.LocalizerBase
import com.qualcomm.robotcore.eventloop.EventLoop
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.OpModeManagerImpl
import com.qualcomm.robotcore.eventloop.opmode.OpModeManagerNotifier
import com.qualcomm.robotcore.util.RobotLog
import java.lang.annotation.Inherited
import java.util.concurrent.Executors

@MustBeDocumented
@Inherited
@Target(AnnotationTarget.CLASS)
@Retention(AnnotationRetention.RUNTIME)
annotation class Pathing

class Apex(var localizer: LocalizerBase, var follower: Follower, var drivetrain: Drivetrain) : OpModeManagerNotifier.Notifications {
    public var debug: Boolean = true
    private var running = false
    private val executor = Executors.newSingleThreadExecutor()

    private lateinit var manager: OpModeManagerImpl


    fun attach(context: Context, eventLoop: EventLoop) {
        if(debug) RobotLog.ii("MooseX", "attachEventLoop: Attached MooseX to Event Loop")
        eventLoop.opModeManager.registerListener(this)
        this.manager = eventLoop.opModeManager
    }

    override fun onOpModePreInit(opMode: OpMode) {
        if (opMode.javaClass.isAnnotationPresent(Pathing::class.java)) {
            localizer.initLocalizer(opMode.hardwareMap)
            drivetrain.setHardwareMap(opMode.hardwareMap)
            drivetrain.initDriveTrain()

            if (!running) {
                running = true
                executor.execute {
                    while (running) {
                        localizer.update()
                    }
                }
            }
        }
    }

    override fun onOpModePreStart(opMode: OpMode) {

    }

    override fun onOpModePostStop(opMode: OpMode) {
        running = false // Stop the background thread
    }
}