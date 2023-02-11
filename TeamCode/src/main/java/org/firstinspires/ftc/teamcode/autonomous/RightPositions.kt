package org.firstinspires.ftc.teamcode.autonomous.right

import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d

@Config
object RightPositions {
    //track width = 12.5
    //w = 15.5
    //l = 17.75
    @JvmField val START = Pose2d(-41.00, 63.00, Math.toRadians(270.00))
    @JvmField val DELIVER = Pose2d(-39.0, 14.0, Math.toRadians(320.0))
    @JvmField val P3 = Vector2d(-59.0, 37.0)
    @JvmField val P2 = Vector2d(-38.5, 37.0)
    @JvmField val P1 = Vector2d(-15.0, 37.0)

}