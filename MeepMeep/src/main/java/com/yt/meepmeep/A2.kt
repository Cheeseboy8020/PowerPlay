package com.yt.meepmeep


import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.noahbres.meepmeep.MeepMeep
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder
import com.noahbres.meepmeep.roadrunner.DriveShim
import com.yt.meepmeep.DriveConstants.*

fun main() {
    System.setProperty("sun.java2d.opengl", "true")
    val meepMeep = MeepMeep(700, 120)

    val bot = DefaultBotBuilder(meepMeep)
        .setDimensions(WIDTH, LENGTH)
        .setConstraints(MAX_VEL, MAX_ACCEL, MAX_ANG_VEL, MAX_ANG_ACCEL, TRACK_WIDTH)
        .setColorScheme(ColorSchemeBlueDark())
        .followTrajectorySequence { drive: DriveShim ->
            drive.trajectorySequenceBuilder(Pose2d(41.00, 63.00, Math.toRadians(270.00)))
            .splineTo(Vector2d(36.55, 53.77), Math.toRadians(254.78))
            .splineTo(Vector2d(35.15, 33.55), Math.toRadians(264.09))
            .splineTo(Vector2d(35.95, 21.93), Math.toRadians(253.78))
            .splineTo(Vector2d(36.35, 14.52), Math.toRadians(245.02))
            .splineToSplineHeading(Pose2d(30.00, 12.00, Math.toRadians(0.00)), Math.toRadians(261.57))
            .build()
        }


    meepMeep
        .setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
        .setTheme(ColorSchemeBlueDark())
        .setBackgroundAlpha(1f)
        .addEntity(bot)
        .start()
}