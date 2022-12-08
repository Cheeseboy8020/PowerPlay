package com.yogictransformers.meepmeep


import com.acmerobotics.roadrunner.geometry.Vector2d
import com.noahbres.meepmeep.MeepMeep
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder
import com.noahbres.meepmeep.roadrunner.DriveShim
import com.yogictransformers.meepmeep.DriveConstants.*
import com.yogictransformers.meepmeep.Positions.A2
import com.yogictransformers.meepmeep.Positions.X2
import com.yogictransformers.meepmeep.Positions.P1

fun main() {
    System.setProperty("sun.java2d.opengl", "true")
    val meepMeep = MeepMeep(650, 60)

    val bot = DefaultBotBuilder(meepMeep)
        .setDimensions(WIDTH, HEIGHT)
        .setConstraints(MAX_VEL, MAX_ACCEL, MAX_ANG_VEL, MAX_ANG_ACCEL, TRACK_WIDTH)
        .setColorScheme(ColorSchemeBlueDark())
        .followTrajectorySequence { drive: DriveShim ->
            drive.trajectorySequenceBuilder(A2)
                .strafeRight(27.0)
                .lineTo(Vector2d(-57.5, 17.0))
                .splineTo(Positions.X2.vec(), Positions.X2.heading)
                .lineTo(Vector2d(-37.5, 11.5))
                .turn(Math.toRadians(45.0))
                .lineTo(P1.vec())
                .turn(Math.toRadians(90.0))
                .build()
        }


    meepMeep
        .setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
        .setTheme(ColorSchemeBlueDark())
        .setBackgroundAlpha(1f)
        .addEntity(bot)
        .start()
}
