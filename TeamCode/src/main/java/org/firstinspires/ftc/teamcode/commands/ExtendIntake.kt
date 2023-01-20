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
class ExtendIntake(private var intake: IntakeExtension, var extPos: Double) : CommandBase() {
    val time = ElapsedTime()
    private var originPos = 0.0

    companion object{
        @JvmField var EXT_POS = 0.29
    }
    init {
        addRequirements(intake)
    }

    override fun initialize() {
        originPos = intake.extLeft.position
        time.reset()
        intake.extend(Pair(extPos, 0.6+extPos))
    }


    override fun isFinished(): Boolean {
        return time.milliseconds() >= abs(
            intake.extLeft.position - originPos) /IntakeExtension.EXT_SPEED + 100
    }
}