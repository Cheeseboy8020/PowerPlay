package org.firstinspires.ftc.teamcode.autonomous.left

import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.roadrunner.geometry.Pose2d

@Config
object Positions {
    //track width = 12.5
    //w = 15.5
    //l = 17.75
    @JvmField val START = Pose2d(40.0, 62.0, Math.toRadians(270.0))
    @JvmField val DELIVER = Pose2d(39.0, 14.0, Math.toRadians(220.0))
    @JvmField val P1 = Pose2d(59.0, 16.0, Math.toRadians(90.0))
    @JvmField val P2 = Pose2d(37.0, 16.0, Math.toRadians(90.0))
    @JvmField val P3 = Pose2d(14.0, 16.0, Math.toRadians(90.0))

}