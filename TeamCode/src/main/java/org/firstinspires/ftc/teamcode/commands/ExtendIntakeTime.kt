package org.firstinspires.ftc.teamcode.commands

import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.alphago.agDistanceLocalization.geometry.Pose
import com.arcrobotics.ftclib.command.CommandBase
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.subsystems.IntakeExtension
import kotlin.math.abs

@Config
class ExtendIntakeTime(private var intake: IntakeExtension, var extPos: Double) : CommandBase() {
    val time = ElapsedTime()
    init {
        addRequirements(intake)
    }

    override fun initialize() {
        time.reset()
        intake.extend(Pair(extPos, 1-extPos))
    }


    override fun isFinished(): Boolean {
        return time.milliseconds()>750
    }
}