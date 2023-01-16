package com.yt.meepmeep


import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.noahbres.meepmeep.MeepMeep
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder
import com.noahbres.meepmeep.roadrunner.DriveShim
import com.yt.meepmeep.DriveConstants.*
import com.yt.meepmeep.Positions.A2
import com.yt.meepmeep.Positions.X2
import com.yt.meepmeep.Positions.P1

fun main() {
    System.setProperty("sun.java2d.opengl", "true")
    val meepMeep = MeepMeep(700, 120)

    val bot = DefaultBotBuilder(meepMeep)
        .setDimensions(WIDTH, LENGTH)
        .setConstraints(MAX_VEL, MAX_ACCEL, MAX_ANG_VEL, MAX_ANG_ACCEL, TRACK_WIDTH)
        .setColorScheme(ColorSchemeBlueDark())
        .followTrajectorySequence { drive: DriveShim ->

            drive.trajectorySequenceBuilder(A2)
                .lineToLinearHeading(Pose2d(-36.0, 24.0, Math.toRadians(270.0)))
                .lineToLinearHeading(Pose2d(-45.3, 5.3, Math.toRadians(166.0)))

                .lineToLinearHeading(Pose2d(-60.0, 16.0, Math.toRadians(90.0))) // loc1
                //.lineToLinearHeading(Pose2d(-36.0, 16.0, Math.toRadians(90.0))) // loc2
//                .lineToLinearHeading(Pose2d(-12.0, 16.0, Math.toRadians(90.0))) // loc3
                .build()
        }


    meepMeep
        .setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
        .setTheme(ColorSchemeBlueDark())
        .setBackgroundAlpha(1f)
        .addEntity(bot)
        .start()
}